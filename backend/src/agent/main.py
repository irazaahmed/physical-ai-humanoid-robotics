"""
Main FastAPI application for the agent service.

This module sets up the FastAPI app with routing and integrates with other components.
"""
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from slowapi.errors import RateLimitExceeded
import os
from dotenv import load_dotenv
from src.agent.rate_limiter import setup_rate_limiter

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics - Agent Service",
    description="Google Gemini agent service providing reasoning capabilities over textbook content",
    version="1.0.0"
)

# Set up rate limiting
setup_rate_limiter(app)

# Register the rate limit exceeded handler
@app.exception_handler(RateLimitExceeded)
async def rate_limit_exceeded_handler(request: Request, exc: RateLimitExceeded):
    return JSONResponse(
        status_code=429,
        content={
            "error": "Rate limit exceeded",
            "message": "Too many requests from your IP address. Please try again later.",
            "retry_after": exc.retry_after,
            "timestamp": __import__('datetime').datetime.utcnow().isoformat()
        }
    )

@app.get("/")
async def root():
    """
    Root endpoint for the agent service.
    """
    return {
        "message": "Welcome to the Physical AI & Humanoid Robotics Agent Service",
        "version": "1.0.0",
        "endpoints": [
            "/api/v1/chat",
            "/api/v1/query",
            "/api/v1/health"
        ]
    }

# Include API routes
from src.agent.api import router as api_router
app.include_router(api_router, prefix="/api/v1")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True
    )