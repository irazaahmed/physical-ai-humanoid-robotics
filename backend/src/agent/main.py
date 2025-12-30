"""
Main FastAPI application for the agent service.

This module sets up the FastAPI app with routing and integrates with other components.
"""
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from slowapi.errors import RateLimitExceeded
import os
from src.agent.config import Config
from src.agent.rate_limiter import setup_rate_limiter

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics - Agent Service",
    description="OpenRouter agent service providing reasoning capabilities over textbook content",
    version="1.0.0"
)

# Add CORS middleware to allow requests from the frontend
# In production, only allow the specific Netlify frontend domain
FRONTEND_URL = os.getenv("FRONTEND_URL", "https://physical-ai-humanoid-robotics-ar.netlify.app")
IS_PRODUCTION = os.getenv("VERCEL_ENV") is not None or os.getenv("ENVIRONMENT") == "production"

if IS_PRODUCTION:
    # Production: Only allow the Netlify frontend
    allow_origins = [FRONTEND_URL]
else:
    # Development: Allow localhost for testing
    allow_origins = [
        "http://localhost:3000",
        "http://localhost:3001",
        "http://localhost:8000",
        "http://localhost:8080",
        "http://127.0.0.1:3000",
        "http://127.0.0.1:3001",
        "http://127.0.0.1:8000",
        "http://127.0.0.1:8080",
        FRONTEND_URL  # Include the production URL for consistency
    ]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allow_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose custom headers if needed
    # expose_headers=["Access-Control-Allow-Origin"]
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
        port=int(os.getenv("PORT", 8001)),
        reload=True
    )