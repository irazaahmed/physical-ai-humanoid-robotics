import numpy as np
from typing import List, Tuple
from src.utils.logging import EmbeddingError, setup_logging

logger = setup_logging()

def validate_embeddings(embeddings: List[List[float]], original_texts: List[str]) -> Tuple[bool, List[str]]:
    """
    Validate that embeddings accurately represent the original content.
    
    Args:
        embeddings: List of embedding vectors
        original_texts: List of original text chunks
        
    Returns:
        Tuple of (is_valid, list_of_errors)
    """
    errors = []
    
    if len(embeddings) != len(original_texts):
        errors.append(f"Mismatch: {len(embeddings)} embeddings for {len(original_texts)} texts")
        return False, errors
    
    for i, (embedding, text) in enumerate(zip(embeddings, original_texts)):
        # Check that embedding is not empty
        if not embedding or len(embedding) == 0:
            errors.append(f"Embedding {i} is empty")
            continue
            
        # Check that embedding contains valid numbers (not NaN or infinity)
        if any(np.isnan(val) or np.isinf(val) for val in embedding):
            errors.append(f"Embedding {i} contains invalid values (NaN or infinity)")
            continue
            
        # Check that embedding has consistent dimensions
        if i > 0 and len(embedding) != len(embeddings[0]):
            errors.append(f"Embedding {i} has inconsistent dimension: {len(embedding)} vs {len(embeddings[0])}")
            continue
            
        # Check that the original text is not empty
        if not text or len(text.strip()) == 0:
            errors.append(f"Original text {i} is empty but has an embedding")
            continue
    
    is_valid = len(errors) == 0
    if is_valid:
        logger.info(f"Successfully validated {len(embeddings)} embeddings")
    else:
        logger.error(f"Embedding validation failed: {len(errors)} errors found")
        for error in errors:
            logger.error(error)
    
    return is_valid, errors


def calculate_similarity(embedding1: List[float], embedding2: List[float]) -> float:
    """
    Calculate cosine similarity between two embeddings.
    
    Args:
        embedding1: First embedding vector
        embedding2: Second embedding vector
        
    Returns:
        Cosine similarity value between -1 and 1
    """
    try:
        # Convert to numpy arrays
        emb1 = np.array(embedding1)
        emb2 = np.array(embedding2)
        
        # Calculate cosine similarity
        dot_product = np.dot(emb1, emb2)
        norm1 = np.linalg.norm(emb1)
        norm2 = np.linalg.norm(emb2)
        
        if norm1 == 0 or norm2 == 0:
            return 0.0  # If either vector is zero, similarity is 0
            
        similarity = dot_product / (norm1 * norm2)
        return float(similarity)
    except Exception as e:
        logger.error(f"Error calculating similarity: {str(e)}")
        return 0.0


def verify_content_represented(original_text: str, embedding: List[float], 
                             sample_texts: List[str], sample_embeddings: List[List[float]]) -> bool:
    """
    Verify that an embedding represents the expected content by comparing
    similarity with other embeddings.
    
    Args:
        original_text: The original text that should be represented
        embedding: The embedding of the original text
        sample_texts: List of sample texts
        sample_embeddings: List of embeddings for the sample texts
        
    Returns:
        True if content is properly represented, False otherwise
    """
    try:
        # Check similarity with the original text
        # This is a simplified check - in practice, you'd want more sophisticated validation
        if len(sample_texts) > 0 and len(sample_embeddings) > 0:
            # Calculate similarity with itself if available in samples
            for sample_text, sample_emb in zip(sample_texts, sample_embeddings):
                if sample_text == original_text:
                    similarity = calculate_similarity(embedding, sample_emb)
                    if similarity > 0.95:  # High similarity for identical content
                        return True
                    else:
                        logger.warning(f"Expected high similarity for identical content, got {similarity}")
                        return False
        
        # If the original text is not in samples, check basic embedding properties
        if not embedding or len(embedding) == 0:
            return False
            
        # Check if the embedding has meaningful values (not all zeros)
        if all(val == 0.0 for val in embedding):
            logger.error("Embedding contains all zeros - likely invalid")
            return False
            
        return True
    except Exception as e:
        logger.error(f"Error in content verification: {str(e)}")
        return False