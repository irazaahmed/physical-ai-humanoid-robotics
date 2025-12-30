"""
Vercel serverless function entrypoint for the FastAPI application.

This module creates a serverless-compatible entrypoint for the FastAPI app
that can be deployed on Vercel's platform using the Mangum adapter.
"""
import os
import sys
from pathlib import Path

# Add the backend directory to the Python path to ensure imports work correctly
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

from mangum import Mangum
from src.agent.main import app  # Import the FastAPI app instance

# Create the Mangum adapter to handle ASGI app in serverless environment
handler = Mangum(app, lifespan="off")