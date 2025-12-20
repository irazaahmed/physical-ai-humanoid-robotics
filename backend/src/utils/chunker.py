# Import all functions from the new chunking module to maintain backward compatibility
from src.utils.chunking import chunk_text, split_into_sentences, split_large_sentence, get_overlap_text, create_chunk_id, calculate_content_hash

# Export all functions to maintain the same interface
__all__ = [
    'chunk_text',
    'split_into_sentences',
    'split_large_sentence',
    'get_overlap_text',
    'create_chunk_id',
    'calculate_content_hash'
]