from typing import List, Tuple
from src.utils.config import Config
from src.utils.logging import ProcessingError, handle_error, setup_logging
from src.utils.text_normalizer import calculate_token_count, normalize_text
import hashlib

logger = setup_logging()

def chunk_text(content: str, content_id: str = "") -> List[Tuple[str, int, str]]:
    """
    Split content into chunks of specified token size with overlap.

    Args:
        content: The content to chunk
        content_id: The ID of the original content (for generating chunk IDs)

    Returns:
        List of tuples containing (chunk_text, token_count, overlap_text)
    """
    try:
        # Check if content is empty or None and handle gracefully
        if not content:
            logger.warning("Empty content provided for chunking")
            return []
        
        # Normalize content first
        normalized_content = normalize_text(content)

        # Calculate total token count
        total_tokens = calculate_token_count(normalized_content)
        logger.info(f"Content has approximately {total_tokens} tokens")

        # If content is already smaller than chunk size, return as single chunk
        if total_tokens <= Config.CHUNK_SIZE:
            logger.info("Content fits in a single chunk")
            return [(normalized_content, total_tokens, "")]

        # Split content into sentences to maintain semantic boundaries
        sentences = split_into_sentences(normalized_content)

        if not sentences:
            logger.warning("No sentences found in content for chunking")
            return [(normalized_content, total_tokens, "")]

        chunks = []
        current_chunk = ""
        current_token_count = 0
        chunk_index = 0

        i = 0
        while i < len(sentences):
            sentence = sentences[i]
            sentence_token_count = calculate_token_count(sentence)

            # If adding this sentence would exceed the chunk size
            if current_token_count + sentence_token_count > Config.CHUNK_SIZE and current_chunk:
                # Calculate overlap from the end of the current chunk
                overlap_text = get_overlap_text(current_chunk, Config.OVERLAP_SIZE)

                # Add the current chunk to results
                chunks.append((current_chunk.strip(), current_token_count, overlap_text))

                # Start a new chunk with overlap
                current_chunk = overlap_text + " " + sentence
                current_token_count = calculate_token_count(current_chunk)

                chunk_index += 1
                i += 1
            # If the sentence itself is too large, we need to split it
            elif sentence_token_count > Config.CHUNK_SIZE:
                # Split the large sentence into smaller parts
                sentence_chunks = split_large_sentence(sentence)

                for part in sentence_chunks:
                    part_token_count = calculate_token_count(part)

                    if current_token_count + part_token_count > Config.CHUNK_SIZE and current_chunk:
                        overlap_text = get_overlap_text(current_chunk, Config.OVERLAP_SIZE)
                        chunks.append((current_chunk.strip(), current_token_count, overlap_text))

                        current_chunk = overlap_text + " " + part
                        current_token_count = calculate_token_count(current_chunk)
                        chunk_index += 1
                    else:
                        if current_chunk:
                            current_chunk += " " + part
                        else:
                            current_chunk = part
                        current_token_count += part_token_count
                i += 1
            else:
                # Add sentence to current chunk
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence
                current_token_count += sentence_token_count
                i += 1

        # Add the final chunk if there's content left
        if current_chunk.strip():
            overlap_text = get_overlap_text(current_chunk, Config.OVERLAP_SIZE)
            chunks.append((current_chunk.strip(), current_token_count, overlap_text))

        logger.info(f"Content chunked into {len(chunks)} chunks")
        return chunks

    except Exception as e:
        handle_error(e, logger, "Text chunking")
        raise ProcessingError(f"Failed to chunk text: {str(e)}")


def split_into_sentences(text: str) -> List[str]:
    """
    Split text into sentences.

    Args:
        text: The text to split

    Returns:
        List of sentences
    """
    # This is a simplified sentence splitter
    # In a production system, you would use a proper NLP library
    import re

    # Split on sentence-ending punctuation followed by whitespace or end of string
    sentences = re.split(r'[.!?]+\s+|(?<=[.!?])\s+(?=[A-Z])', text)

    # Clean up sentences
    sentences = [s.strip() for s in sentences if s.strip()]

    return sentences


def split_large_sentence(sentence: str) -> List[str]:
    """
    Split a large sentence that exceeds chunk size into smaller parts.

    Args:
        sentence: The sentence to split

    Returns:
        List of sentence parts
    """
    words = sentence.split()
    parts = []
    current_part = []
    current_tokens = 0

    # Target token count for large sentence parts (slightly less than full chunk to allow buffer)
    target_tokens = Config.CHUNK_SIZE - 50

    for word in words:
        word_tokens = calculate_token_count(word)

        if current_tokens + word_tokens > target_tokens and current_part:
            # Add current part to results
            parts.append(" ".join(current_part))
            current_part = [word]
            current_tokens = word_tokens
        else:
            current_part.append(word)
            current_tokens += word_tokens

    # Add the last part if it exists
    if current_part:
        parts.append(" ".join(current_part))

    return parts


def get_overlap_text(text: str, overlap_size: int) -> str:
    """
    Extract the overlap text from the end of the chunk.

    Args:
        text: The text to extract overlap from
        overlap_size: The desired overlap size in tokens

    Returns:
        Overlap text
    """
    if not text or overlap_size <= 0:
        return ""

    words = text.split()
    if not words:
        return ""

    # Start from the end and accumulate words until we reach the desired overlap token count
    overlap_words = []
    current_tokens = 0

    # Process words in reverse
    for word in reversed(words):
        word_tokens = calculate_token_count(word)

        if current_tokens + word_tokens <= overlap_size:
            overlap_words.insert(0, word)  # Insert at the beginning to maintain order
            current_tokens += word_tokens
        else:
            break  # Can't fit this word without exceeding the limit

    return " ".join(overlap_words)


def create_chunk_id(content_id: str, chunk_index: int) -> str:
    """
    Create a unique ID for a chunk.

    Args:
        content_id: The ID of the parent content
        chunk_index: The index of this chunk

    Returns:
        Unique chunk ID
    """
    return f"{content_id}_chunk_{chunk_index:03d}"


def calculate_content_hash(text: str) -> str:
    """
    Calculate a hash for the content to enable duplicate detection.

    Args:
        text: The text to hash

    Returns:
        SHA256 hash of the content
    """
    if not text:
        return ""
    return hashlib.sha256(text.encode('utf-8')).hexdigest()