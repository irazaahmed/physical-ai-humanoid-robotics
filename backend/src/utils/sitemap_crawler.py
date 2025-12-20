import requests
from bs4 import BeautifulSoup
from typing import List
from src.utils.config import Config
from src.utils.logging import setup_logging
import time

logger = setup_logging()

def fetch_sitemap_urls(sitemap_url: str = None) -> List[str]:
    """
    Fetch URLs from the sitemap.xml file and filter for documentation pages.
    
    Args:
        sitemap_url: The sitemap URL to fetch. If None, uses the configured TEXTBOOK_BASE_URL
        
    Returns:
        List of documentation URLs from the sitemap
    """
    if sitemap_url is None:
        sitemap_url = f"{Config.TEXTBOOK_BASE_URL}/sitemap.xml"
    
    logger.info(f"Fetching sitemap from: {sitemap_url}")
    
    try:
        response = requests.get(sitemap_url, timeout=30)
        response.raise_for_status()
        
        # Parse the sitemap XML
        soup = BeautifulSoup(response.content, 'xml')
        
        # Find all <loc> tags which contain URLs
        loc_tags = soup.find_all('loc')
        
        all_urls = []
        for loc in loc_tags:
            url = loc.get_text().strip()
            if url:
                all_urls.append(url)
        
        # Filter only documentation URLs that contain /docs/
        doc_urls = []
        for url in all_urls:
            if '/docs/' in url:
                doc_urls.append(url)
        
        logger.info(f"Discovered {len(all_urls)} total URLs from sitemap")
        logger.info(f"Filtered to {len(doc_urls)} documentation URLs containing '/docs/'")
        
        # Log a few examples
        for i, url in enumerate(doc_urls[:5]):  # Show first 5 URLs
            logger.info(f"  Example URL {i+1}: {url}")
        
        if len(doc_urls) > 5:
            logger.info(f"  ... and {len(doc_urls) - 5} more URLs")
        
        return doc_urls
        
    except requests.exceptions.RequestException as e:
        logger.error(f"Error fetching sitemap: {str(e)}")
        return []
    except Exception as e:
        logger.error(f"Unexpected error while parsing sitemap: {str(e)}")
        return []

def filter_unique_urls(urls: List[str]) -> List[str]:
    """
    Remove duplicate URLs from the list while preserving order.
    
    Args:
        urls: List of URLs to filter
        
    Returns:
        List of unique URLs
    """
    seen = set()
    unique_urls = []
    
    for url in urls:
        if url not in seen:
            seen.add(url)
            unique_urls.append(url)
    
    logger.info(f"Filtered out {len(urls) - len(unique_urls)} duplicate URLs")
    return unique_urls