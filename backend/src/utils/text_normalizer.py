import re
from typing import List, Tuple
from src.utils.config import Config
from src.utils.logging import ProcessingError, handle_error, setup_logging

logger = setup_logging()

def normalize_text(text: str) -> str:
    """
    Normalize text according to constitution requirements
    
    Args:
        text: The raw text to normalize
        
    Returns:
        The normalized text
    """
    try:
        # Remove excessive whitespace while preserving single spaces
        normalized = re.sub(r'\s+', ' ', text)
        
        # Remove special characters that might interfere with tokenization
        # but preserve important punctuation
        normalized = re.sub(r'[^\w\s\.\,\!\?\;\:\-\(\)\'\"\\\/\[\]\{\}]', ' ', normalized)
        
        # Ensure proper spacing around punctuation
        normalized = re.sub(r'\s*([.,!?;:])\s*', r'\1 ', normalized)
        
        # Remove extra spaces again after punctuation processing
        normalized = re.sub(r'\s+', ' ', normalized).strip()
        
        logger.info("Text normalization completed")
        return normalized
    except Exception as e:
        handle_error(e, logger, "Text normalization")
        raise ProcessingError(f"Failed to normalize text: {str(e)}")


def calculate_token_count(text: str) -> int:
    """
    Calculate the approximate token count for the text.
    For this implementation, we'll use a simple heuristic (words/0.75)
    In a production system, you might use tiktoken or other tokenizers.
    
    Args:
        text: The text to count tokens for
        
    Returns:
        Approximate token count
    """
    try:
        # Simple heuristic: 1 token ~ 0.75 words in English
        # This is a rough approximation - in a real system, use a proper tokenizer like tiktoken
        words = len(text.split())
        # Using 0.75 as rough average since tokens are often subwords
        token_estimate = max(1, int(words / 0.75))
        
        return token_estimate
    except Exception as e:
        handle_error(e, logger, "Token counting")
        raise ProcessingError(f"Failed to calculate token count: {str(e)}")


def extract_section_info(text: str) -> str:
    """
    Extract section information from text if available
    
    Args:
        text: The text to extract section info from
        
    Returns:
        Section name or empty string if not found
    """
    try:
        # Look for common section patterns in the first part of the text
        lines = text.split('\n')[:5]  # Only look at first few lines
        
        for line in lines:
            line = line.strip()
            # Look for patterns like "## Section Name" or "### Chapter Name"
            if line.startswith(('##', '###')) and len(line) < 100:
                section = line.lstrip('# ')
                if section:
                    return section
                    
        # If no markdown headers found, try to extract from regular text
        if lines:
            first_line = lines[0].strip()
            # If the first line is relatively short and looks like a title
            if 5 < len(first_line) < 100 and first_line[0].isupper() and not first_line.endswith('.'):
                return first_line
    
        return ""
    except Exception as e:
        handle_error(e, logger, "Section info extraction")
        return ""