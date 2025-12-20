import logging
import sys
from typing import Any

def setup_logging(level: int = logging.INFO) -> logging.Logger:
    """
    Set up logging configuration for the application.
    
    Args:
        level: Logging level (default: INFO)
        
    Returns:
        Configured logger instance
    """
    # Create logger
    logger = logging.getLogger("rag_embeddings_pipeline")
    logger.setLevel(level)
    
    # Prevent adding multiple handlers if logger already exists
    if logger.handlers:
        return logger
    
    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    
    # Create file handler
    file_handler = logging.FileHandler("rag_embeddings_pipeline.log")
    file_handler.setLevel(level)
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)
    file_handler.setFormatter(formatter)
    
    # Add handlers to logger
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)
    
    return logger


class PipelineError(Exception):
    """Base exception class for pipeline errors"""
    pass


class CrawlerError(PipelineError):
    """Exception raised for errors in the crawling process"""
    pass


class ProcessingError(PipelineError):
    """Exception raised for errors in the content processing"""
    pass


class EmbeddingError(PipelineError):
    """Exception raised for errors in the embedding generation"""
    pass


class StorageError(PipelineError):
    """Exception raised for errors in the storage process"""
    pass


def handle_error(error: Exception, logger: logging.Logger, task_name: str) -> None:
    """
    Standardized error handling function
    
    Args:
        error: The exception that occurred
        logger: Logger instance to log the error
        task_name: Name of the task where error occurred
    """
    logger.error(f"Error in {task_name}: {str(error)}", exc_info=True)
    # In a real implementation, you might want to implement retry logic here
    raise error