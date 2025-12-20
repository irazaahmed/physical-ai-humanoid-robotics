from typing import List
from .models import RetrievedChunk
from .logging import ValidationError, setup_logging

logger = setup_logging()

def validate_similarity_scores(results: List[RetrievedChunk], threshold: float = 0.0) -> List[str]:
    """
    Validate that similarity scores are within acceptable bounds and above threshold.
    
    Args:
        results: List of retrieved chunks with similarity scores
        threshold: Minimum similarity score required
        
    Returns:
        List of validation issues found
    """
    issues = []
    
    for i, chunk in enumerate(results):
        score = chunk.similarity_score
        
        # Check if score is within valid range (0.0 to 1.0)
        if score < 0.0 or score > 1.0:
            issues.append(f"Chunk {i}: Similarity score {score} is outside valid range [0.0, 1.0]")
        
        # Check if score meets threshold
        if score < threshold:
            issues.append(f"Chunk {i}: Similarity score {score} is below threshold {threshold}")
    
    return issues


def validate_metadata_completeness(results: List[RetrievedChunk]) -> List[str]:
    """
    Validate that the metadata is complete for each retrieved result.
    
    Args:
        results: List of retrieved chunks with metadata
        
    Returns:
        List of validation issues found
    """
    issues = []
    
    for i, chunk in enumerate(results):
        md = chunk.metadata
        
        # Check required metadata fields
        if not md.url:
            issues.append(f"Chunk {i}: Missing URL in metadata")
        if not md.module:
            issues.append(f"Chunk {i}: Missing module in metadata")
        if not md.chapter:
            issues.append(f"Chunk {i}: Missing chapter in metadata")
        if not md.section:
            issues.append(f"Chunk {i}: Missing section in metadata")
        if md.chunk_index is None:
            issues.append(f"Chunk {i}: Missing chunk_index in metadata")
    
    return issues


def validate_embedding_dimension(embedding: List[float], expected_dim: int) -> List[str]:
    """
    Validate that the embedding vector has the expected dimensionality.
    
    Args:
        embedding: The embedding vector to validate
        expected_dim: The expected dimension size
        
    Returns:
        List of validation issues found
    """
    issues = []
    
    if not embedding:
        issues.append("Embedding vector is empty")
    elif len(embedding) != expected_dim:
        issues.append(f"Embedding dimension mismatch: expected {expected_dim}, got {len(embedding)}")
    
    return issues


def validate_retrieval_results(results: List[RetrievedChunk], min_similarity_threshold: float = 0.0) -> List[str]:
    """
    Perform comprehensive validation of retrieval results.
    
    Args:
        results: List of retrieved chunks
        min_similarity_threshold: Minimum similarity score threshold
        
    Returns:
        List of all validation issues found
    """
    all_issues = []
    
    # Validate similarity scores
    score_issues = validate_similarity_scores(results, min_similarity_threshold)
    all_issues.extend(score_issues)
    
    # Validate metadata completeness
    metadata_issues = validate_metadata_completeness(results)
    all_issues.extend(metadata_issues)
    
    # Log validation summary
    if all_issues:
        logger.warning(f"Validation found {len(all_issues)} issues in retrieval results")
        for issue in all_issues:
            logger.debug(f"  - {issue}")
    else:
        logger.info("All retrieval results passed validation checks")
    
    return all_issues


def validate_query_text(query: str) -> List[str]:
    """
    Validate the query text for basic requirements.
    
    Args:
        query: The query text to validate
        
    Returns:
        List of validation issues found
    """
    issues = []
    
    if not query or len(query.strip()) == 0:
        issues.append("Query text is empty or contains only whitespace")
    elif len(query.strip()) < 3:
        issues.append("Query text is too short (minimum 3 characters)")
    elif len(query.strip()) > 10000:  # Arbitrary max size
        issues.append("Query text is too long (maximum 10000 characters)")
    
    return issues