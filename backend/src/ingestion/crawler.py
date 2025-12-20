import requests
from bs4 import BeautifulSoup
from typing import List, Dict, Optional
from src.utils.config import Config
from src.utils.logging import CrawlerError, handle_error, setup_logging
import time

logger = setup_logging()

class WebCrawler:
    """Web crawler for extracting content from textbook URLs"""
    
    def __init__(self):
        """Initialize the web crawler"""
        self.session = requests.Session()
        # Set a user agent to avoid being blocked by some websites
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        })
        
    def fetch_page_content(self, url: str, max_retries: int = 3, timeout: int = 10) -> Optional[str]:
        """
        Fetch content from a single URL with retry logic

        Args:
            url: The URL to fetch
            max_retries: Maximum number of retry attempts
            timeout: Request timeout in seconds

        Returns:
            HTML content of the page or None if failed
        """
        for attempt in range(max_retries):
            try:
                response = self.session.get(url, timeout=timeout)

                # Check if we get a 404 or other error status
                if response.status_code == 404:
                    logger.warning(f"Page not found (404): {url}")
                    return None
                elif response.status_code >= 400:
                    logger.warning(f"HTTP error {response.status_code} for {url}")
                    if attempt < max_retries - 1:
                        time.sleep(2 ** attempt)  # Exponential backoff before retry
                        continue
                    else:
                        return None
                elif response.status_code != 200:
                    logger.info(f"Non-200 status code {response.status_code} for {url}, but continuing")

                logger.info(f"Successfully fetched content from {url}")
                return response.text

            except requests.exceptions.RequestException as e:
                logger.warning(f"Attempt {attempt + 1} failed to fetch {url}: {str(e)}")
                if attempt < max_retries - 1:
                    time.sleep(2 ** attempt)  # Exponential backoff
                else:
                    handle_error(e, logger, f"Fetching content from {url}")
                    return None
            except Exception as e:
                logger.error(f"Unexpected error when fetching {url}: {str(e)}")
                if attempt < max_retries - 1:
                    time.sleep(2 ** attempt)  # Exponential backoff
                else:
                    handle_error(e, logger, f"Fetching content from {url}")
                    return None

        return None
    
    def extract_module_chapter_from_url(self, url: str) -> tuple[str, str]:
        """
        Extract module and chapter information from the URL
        
        Args:
            url: The textbook page URL
            
        Returns:
            Tuple of (module, chapter) extracted from the URL
        """
        # This is a simplified implementation - in a real scenario, 
        # you would have a more robust way to extract module/chapter info
        parts = url.replace(Config.TEXTBOOK_BASE_URL, "").strip("/").split("/")
        
        module = "Module Unknown"  # Default value
        chapter = "Chapter Unknown"  # Default value
        
        if len(parts) >= 2:
            # Extract module and chapter from URL path (e.g., /module1/chapter1)
            if "module" in parts[0].lower():
                module = f"Module {parts[0][-1].upper() if parts[0][-1].isdigit() else parts[0]}"
            if parts[1]:
                chapter = parts[1].replace("-", " ").title()
        elif len(parts) >= 1 and parts[0]:
            chapter = parts[0].replace("-", " ").title()
        
        return module, chapter
    
    def extract_text_content(self, html_content: str, url: str) -> tuple[str, str, str]:
        """
        Extract clean text content from HTML, along with module and chapter info

        Args:
            html_content: HTML content to extract text from
            url: The URL the content came from

        Returns:
            Tuple of (clean text, module, chapter)
        """
        try:
            soup = BeautifulSoup(html_content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Process different types of content while preserving structure
            content_parts = []

            # Process content in document order
            for element in soup.find_all(['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'code', 'pre', 'blockquote']):
                tag = element.name
                text = element.get_text().strip()

                if text:  # Only add non-empty text
                    if tag.startswith('h'):  # Heading tags
                        content_parts.append(f"\n### {text}\n")
                    elif tag == 'li':  # List items
                        content_parts.append(f"• {text}\n")
                    elif tag in ['code', 'pre']:  # Code blocks
                        content_parts.append(f"\n```\n{text}\n```\n")
                    elif tag == 'blockquote':  # Block quotes
                        content_parts.append(f"> {text}\n")
                    else:  # Regular paragraphs
                        content_parts.append(f"{text} ")

            # If no structured content found, fall back to basic text extraction
            clean_text = ' '.join(content_parts).strip()

            if not clean_text:
                # Fallback to basic extraction
                text = soup.get_text()
                lines = (line.strip() for line in text.splitlines())
                chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
                clean_text = ' '.join(chunk for chunk in chunks if chunk)

            # Extract module and chapter info from the URL
            module, chapter = self.extract_module_chapter_from_url(url)

            # Try to extract section title from the page
            section = ""
            # Look for common heading tags that might contain section information
            for tag in soup.find_all(['h1', 'h2', 'h3']):
                if tag.get_text().strip():
                    section = tag.get_text().strip()
                    break

            return clean_text, module, chapter

        except Exception as e:
            handle_error(e, logger, f"Extracting text content from {url}")
            return "", "Module Unknown", "Chapter Unknown"
    
    def crawl_urls(self, urls: List[str]) -> List[Dict[str, str]]:
        """
        Crawl multiple URLs and extract their content

        Args:
            urls: List of URLs to crawl

        Returns:
            List of dictionaries containing URL, content, module, and chapter info
        """
        results = []
        skipped_count = 0
        success_count = 0

        for i, url in enumerate(urls):
            logger.info(f"Crawling {i+1}/{len(urls)}: {url}")

            html_content = self.fetch_page_content(url)
            if html_content:
                clean_text, module, chapter = self.extract_text_content(html_content, url)

                if clean_text:
                    results.append({
                        'url': url,
                        'content': clean_text,
                        'module': module,
                        'chapter': chapter
                    })
                    logger.info(f"Successfully extracted content from {url}")
                    success_count += 1
                else:
                    logger.warning(f"No clean text extracted from {url}")
                    skipped_count += 1
            else:
                logger.warning(f"Failed to fetch content from {url}")
                skipped_count += 1

        logger.info(f"Crawling completed. Successfully processed {success_count} URLs, {skipped_count} URLs skipped.")
        logger.info(f"Total URLs processed: {len(results)} out of {len(urls)}")
        return results