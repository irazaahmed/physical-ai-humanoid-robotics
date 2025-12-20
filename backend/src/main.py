import os
from typing import List, Dict, Any
from src.ingestion.crawler import WebCrawler
from src.utils.text_normalizer import normalize_text
from src.utils.chunking import chunk_text, create_chunk_id, calculate_content_hash
from src.embedding.cohere_client import CohereClient
from src.storage.qdrant_client import QdrantStorage
from src.ingestion.idempotent_ingestor import IdempotentIngestion
from src.models.data_models import TextbookContent, ContentChunk, EmbeddingVector
from src.utils.config import Config
from src.utils.logging import setup_logging
from src.utils.sitemap_crawler import fetch_sitemap_urls, filter_unique_urls
import uuid

logger = setup_logging()

class EmbeddingPipeline:
    """Main orchestration class for the embedding pipeline"""
    
    def __init__(self):
        """Initialize all components of the pipeline"""
        # Validate configuration
        config_errors = Config.validate()
        if config_errors:
            raise ValueError(f"Configuration validation failed: {', '.join(config_errors)}")
        
        # Initialize components
        self.crawler = WebCrawler()
        self.cohere_client = CohereClient()
        self.qdrant_storage = QdrantStorage()
        self.idempotent_ingestor = IdempotentIngestion(self.qdrant_storage)
        
        # Create Qdrant collection
        self.qdrant_storage.create_collection()
    
    def run_pipeline(self, urls: List[str]) -> bool:
        """
        Run the complete embedding pipeline: crawl → normalize → chunk → embed → store
        
        Args:
            urls: List of textbook URLs to process
            
        Returns:
            True if pipeline completed successfully, False otherwise
        """
        try:
            logger.info(f"Starting pipeline for {len(urls)} URLs")
            
            # Step 1: Crawl URLs
            logger.info("Step 1: Crawling URLs...")
            crawled_contents = self.crawler.crawl_urls(urls)
            
            if not crawled_contents:
                logger.error("No content crawled from URLs")
                return False
            
            # Step 2: Process each crawled content
            all_vectors = []
            all_payloads = []
            
            logger.info(f"Step 2: Processing {len(crawled_contents)} crawled contents...")
            
            for i, crawled in enumerate(crawled_contents):
                logger.info(f"Processing content {i+1}/{len(crawled_contents)}: {crawled['url']}")
                
                # Normalize content
                normalized_content = normalize_text(crawled['content'])
                
                # Chunk content according to constitution (512 tokens with 64-token overlap)
                chunks_data = chunk_text(normalized_content, crawled['url'])

                # Log number of chunks created
                logger.info(f"Created {len(chunks_data)} chunks for URL: {crawled['url']}")

                # Process each chunk
                content_chunks = []
                for chunk_idx, (chunk_text_val, token_count, overlap_text) in enumerate(chunks_data):
                    # Create chunk ID
                    chunk_id = create_chunk_id(crawled['url'], chunk_idx)

                    # Calculate content hash for idempotency
                    chunk_hash = calculate_content_hash(chunk_text_val)

                    content_chunk = {
                        'id': chunk_id,
                        'content_id': crawled['url'],
                        'chunk_index': chunk_idx,
                        'text': chunk_text_val,
                        'token_count': token_count,
                        'overlap_text': overlap_text,
                        'hash': chunk_hash
                    }
                    content_chunks.append(content_chunk)
                
                # Prepare texts for embedding
                texts_to_embed = [chunk['text'] for chunk in content_chunks if chunk['text'].strip()]
                
                if not texts_to_embed:
                    logger.warning(f"No text to embed for {crawled['url']}")
                    continue
                
                # Step 3: Generate embeddings
                logger.info(f"Step 3: Generating embeddings for {len(texts_to_embed)} chunks from {crawled['url']}...")
                if texts_to_embed:  # Only generate embeddings if we have text to embed
                    embeddings = self.cohere_client.generate_embeddings(texts_to_embed)
                else:
                    logger.warning(f"No text to embed from {crawled['url']}")
                    continue  # Skip to the next crawled content if no text is available
                
                # Step 4: Prepare payloads for storage with metadata (using deterministic UUIDs for IDs)
                import hashlib
                for j, (chunk, embedding) in enumerate(zip(content_chunks, embeddings)):
                    # Create deterministic UUID based on URL and chunk index for idempotent ingestion
                    unique_identifier = f"{crawled['url']}_chunk_{chunk['chunk_index']}"
                    # Create deterministic UUID using uuid5 with a namespace and the unique identifier
                    deterministic_uuid = str(uuid.uuid5(uuid.NAMESPACE_DNS, unique_identifier))

                    payload = {
                        'id': deterministic_uuid,  # Use deterministic UUID instead of URL-based ID
                        'module': crawled['module'],
                        'chapter': crawled['chapter'],
                        'section': '',  # Could extract from content if available
                        'url': crawled['url'],
                        'chunk_index': chunk['chunk_index'],
                        'hash': chunk['hash'],
                        'content_id': chunk['content_id'],
                        'content': chunk['text']  # Include the actual content text for retrieval
                    }

                    all_vectors.append(embedding)
                    all_payloads.append(payload)
            
            # Step 5: Store vectors with idempotency
            logger.info(f"Step 4: Storing {len(all_vectors)} vectors with idempotency...")
            success = self.idempotent_ingestor.process_ingestion_with_idempotency(
                all_vectors, 
                all_payloads
            )
            
            if success:
                logger.info("Pipeline completed successfully")
                
                # Final validation
                validation_success, validation_result = self.qdrant_storage.validate_storage()
                if validation_success:
                    logger.info(f"Final validation passed: {validation_result}")
                else:
                    logger.error(f"Final validation failed: {validation_result}")
                
                return True
            else:
                logger.error("Pipeline failed during storage step")
                return False
                
        except Exception as e:
            logger.error(f"Pipeline failed with error: {str(e)}", exc_info=True)
            return False


def main():
    """Main function to run the embedding pipeline"""
    try:
        # Initialize the pipeline
        pipeline = EmbeddingPipeline()

        # Validate that URLs are set in config
        if not Config.TEXTBOOK_BASE_URL:
            logger.error("TEXTBOOK_BASE_URL not set in configuration")
            return False

        # Fetch documentation URLs from the sitemap
        logger.info("Fetching documentation URLs from sitemap...")
        urls = fetch_sitemap_urls()

        if not urls:
            logger.error("No documentation URLs found in sitemap")
            return False

        # Filter out any duplicate URLs
        unique_urls = filter_unique_urls(urls)

        logger.info(f"Starting pipeline for {len(unique_urls)} unique documentation URLs")

        # Run the pipeline with the sitemap-derived URLs
        success = pipeline.run_pipeline(unique_urls)

        if success:
            logger.info("Embedding pipeline completed successfully!")
            return True
        else:
            logger.error("Embedding pipeline failed!")
            return False

    except Exception as e:
        logger.error(f"Error running pipeline: {str(e)}", exc_info=True)
        return False


def main():
    """Main function to run the embedding pipeline"""
    try:
        # Check if we should start the agent service
        if os.getenv("START_AGENT_SERVICE", "false").lower() == "true":
            # Start the agent service
            import uvicorn
            from src.agent.main import app as agent_app
            print("Starting Agent service on port 8001...")
            uvicorn.run(
                agent_app,
                host="0.0.0.0",
                port=8001,
                reload=False
            )
            return True

        # Initialize the pipeline
        pipeline = EmbeddingPipeline()

        # Validate that URLs are set in config
        if not Config.TEXTBOOK_BASE_URL:
            logger.error("TEXTBOOK_BASE_URL not set in configuration")
            return False

        # Fetch documentation URLs from the sitemap
        logger.info("Fetching documentation URLs from sitemap...")
        urls = fetch_sitemap_urls()

        if not urls:
            logger.error("No documentation URLs found in sitemap")
            return False

        # Filter out any duplicate URLs
        unique_urls = filter_unique_urls(urls)

        logger.info(f"Starting pipeline for {len(unique_urls)} unique documentation URLs")

        # Run the pipeline with the sitemap-derived URLs
        success = pipeline.run_pipeline(unique_urls)

        if success:
            logger.info("Embedding pipeline completed successfully!")
            return True
        else:
            logger.error("Embedding pipeline failed!")
            return False

    except Exception as e:
        logger.error(f"Error running pipeline: {str(e)}", exc_info=True)
        return False


if __name__ == "__main__":
    main()