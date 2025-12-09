from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.v1.routes import chat, auth, ingest
from app.core.settings import settings

app = FastAPI(
    title="Physical AI Humanoid Robotics Textbook RAG API",
    description="Backend API for the Physical AI Humanoid Robotics Textbook with RAG capabilities",
    version="1.0.0"
)

# Add CORS middleware to allow requests from GitHub Pages
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://osamabinadnan.github.io"],  # GitHub Pages URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
app.include_router(auth.router, prefix="/api/v1", tags=["auth"])
app.include_router(ingest.router, prefix="/api/v1", tags=["ingest"])

@app.get("/")
def read_root():
    return {"message": "Physical AI Humanoid Robotics Textbook RAG API"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "backend-api"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
