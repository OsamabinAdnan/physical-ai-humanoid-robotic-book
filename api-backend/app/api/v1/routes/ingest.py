from fastapi import APIRouter, HTTPException, status
from typing import List
from app.utils.embeddings import embedding_generator
from app.utils.qdrant_client import qdrant_manager
from app.utils.logging import log_api_call, log_error
from app.core.settings import settings
import json


router = APIRouter()


@router.post("/ingest/textbook")
def ingest_textbook_content(text_chunks: List[dict]):
    """
    Ingest textbook content into the vector database
    Each chunk should have 'text', 'metadata' (including chapter, section, etc.)
    """
    try:
        # Log the API call
        log_api_call(
            endpoint="/ingest/textbook",
            method="POST",
            params={
                "chunks_count": len(text_chunks)
            }
        )

        # Ensure the collection exists
        qdrant_manager.create_collection()

        # Process each text chunk
        successful_chunks = 0
        for chunk in text_chunks:
            text = chunk.get("text", "")
            metadata = chunk.get("metadata", {})

            if not text:
                continue  # Skip empty chunks

            # Generate embedding for the text
            embedding = embedding_generator.generate_embedding(text)

            # Store in Qdrant with metadata
            point_id = qdrant_manager.store_text_chunk(
                text=text,
                metadata=metadata,
                vector=embedding
            )

            successful_chunks += 1
            print(f"Stored chunk with ID: {point_id}")

        # Log successful API call
        log_api_call(
            endpoint="/ingest/textbook",
            method="POST",
            params={
                "chunks_count": len(text_chunks)
            },
            response={
                "successful_chunks": successful_chunks
            }
        )

        return {
            "status": "success",
            "message": f"Successfully ingested {successful_chunks} text chunks out of {len(text_chunks)} provided"
        }

    except Exception as e:
        # Log the error
        log_error(
            error_msg=str(e),
            context={
                "chunks_count": len(text_chunks) if 'text_chunks' in locals() else 0
            }
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while ingesting the textbook content"
        )


@router.post("/ingest/chapter")
def ingest_chapter(chapter_data: dict):
    """
    Ingest a single chapter with its sections and content
    chapter_data should contain: title, sections, content, etc.
    """
    try:
        # Log the API call
        log_api_call(
            endpoint="/ingest/chapter",
            method="POST",
            params={
                "chapter_title": chapter_data.get("title", "unknown"),
                "sections_count": len(chapter_data.get("sections", []))
            }
        )

        # Ensure the collection exists
        qdrant_manager.create_collection()

        chapter_title = chapter_data.get("title", "")
        sections = chapter_data.get("sections", [])
        chapter_content = chapter_data.get("content", "")
        total_items_processed = 0

        # Process the main chapter content if provided
        if chapter_content:
            # Break down the content into smaller chunks if needed
            # For now, we'll store the entire content as one chunk with chapter metadata
            embedding = embedding_generator.generate_embedding(chapter_content)

            metadata = {
                "chapter_title": chapter_title,
                "section": "main_content",
                "type": "chapter_content"
            }

            point_id = qdrant_manager.store_text_chunk(
                text=chapter_content,
                metadata=metadata,
                vector=embedding
            )

            total_items_processed += 1
            print(f"Stored chapter content with ID: {point_id}")

        # Process each section if provided
        for section in sections:
            section_title = section.get("title", "")
            section_content = section.get("content", "")

            if section_content:
                # Generate embedding for the section content
                embedding = embedding_generator.generate_embedding(section_content)

                metadata = {
                    "chapter_title": chapter_title,
                    "section_title": section_title,
                    "type": "section_content"
                }

                point_id = qdrant_manager.store_text_chunk(
                    text=section_content,
                    metadata=metadata,
                    vector=embedding
                )

                total_items_processed += 1
                print(f"Stored section '{section_title}' with ID: {point_id}")

        # Log successful API call
        log_api_call(
            endpoint="/ingest/chapter",
            method="POST",
            params={
                "chapter_title": chapter_data.get("title", "unknown"),
                "sections_count": len(chapter_data.get("sections", []))
            },
            response={
                "total_items_processed": total_items_processed
            }
        )

        return {
            "status": "success",
            "message": f"Successfully ingested chapter: {chapter_title}",
            "chapter_title": chapter_title,
            "sections_processed": len(sections),
            "total_items_processed": total_items_processed
        }

    except Exception as e:
        # Log the error
        log_error(
            error_msg=str(e),
            context={
                "chapter_title": chapter_data.get("title", "unknown") if 'chapter_data' in locals() else 'unknown'
            }
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while ingesting the chapter"
        )


@router.post("/ingest/reset")
def reset_vector_store():
    """
    Delete the existing collection and recreate it (for re-indexing)
    """
    try:
        # Log the API call
        log_api_call(
            endpoint="/ingest/reset",
            method="POST"
        )

        qdrant_manager.delete_collection()

        # Recreate the collection
        qdrant_manager.create_collection()

        # Log successful API call
        log_api_call(
            endpoint="/ingest/reset",
            method="POST",
            response={
                "status": "success",
                "message": "Vector store reset successfully"
            }
        )

        return {
            "status": "success",
            "message": "Vector store reset successfully"
        }

    except Exception as e:
        # Log the error
        log_error(
            error_msg=str(e),
            context={
                "operation": "vector_store_reset"
            }
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while resetting the vector store"
        )


@router.get("/ingest/status")
def get_ingestion_status():
    """
    Get the status of the vector database
    """
    try:
        # Log the API call
        log_api_call(
            endpoint="/ingest/status",
            method="GET"
        )

        # Get collection info from Qdrant
        collection_info = qdrant_manager.client.get_collection(qdrant_manager.collection_name)

        response_data = {
            "status": "success",
            "collection_name": qdrant_manager.collection_name,
            "vectors_count": collection_info.points_count,
            "indexed": True
        }

        # Log successful API call
        log_api_call(
            endpoint="/ingest/status",
            method="GET",
            response=response_data
        )

        return response_data

    except Exception as e:
        # The collection might not exist yet
        response_data = {
            "status": "pending",
            "collection_name": qdrant_manager.collection_name,
            "vectors_count": 0,
            "indexed": False,
            "message": "Collection does not exist yet. Run ingestion to create it."
        }

        # Log successful API call (even though there was an exception, it's expected behavior)
        log_api_call(
            endpoint="/ingest/status",
            method="GET",
            response=response_data
        )

        return response_data


@router.post("/ingest/textbook-full")
def ingest_full_textbook(textbook_data: dict):
    """
    Ingest an entire textbook with multiple chapters and sections
    textbook_data should contain: title, description, chapters (list of chapter objects)
    Each chapter object should have: title, content, sections
    """
    try:
        # Log the API call
        log_api_call(
            endpoint="/ingest/textbook-full",
            method="POST",
            params={
                "textbook_title": textbook_data.get("title", "unknown"),
                "chapters_count": len(textbook_data.get("chapters", []))
            }
        )

        # Ensure the collection exists
        qdrant_manager.create_collection()

        textbook_title = textbook_data.get("title", "")
        textbook_description = textbook_data.get("description", "")
        chapters = textbook_data.get("chapters", [])

        total_processed = 0
        total_chapters = len(chapters)

        for i, chapter in enumerate(chapters):
            chapter_title = chapter.get("title", f"Chapter {i+1}")
            chapter_content = chapter.get("content", "")
            sections = chapter.get("sections", [])

            # Process the main chapter content if provided
            if chapter_content:
                embedding = embedding_generator.generate_embedding(chapter_content)

                metadata = {
                    "textbook_title": textbook_title,
                    "textbook_description": textbook_description,
                    "chapter_title": chapter_title,
                    "section": "main_content",
                    "type": "chapter_content",
                    "chapter_number": i + 1
                }

                point_id = qdrant_manager.store_text_chunk(
                    text=chapter_content,
                    metadata=metadata,
                    vector=embedding
                )

                total_processed += 1
                print(f"Stored chapter '{chapter_title}' content with ID: {point_id}")

            # Process each section if provided
            for j, section in enumerate(sections):
                section_title = section.get("title", f"Section {j+1}")
                section_content = section.get("content", "")

                if section_content:
                    # Generate embedding for the section content
                    embedding = embedding_generator.generate_embedding(section_content)

                    metadata = {
                        "textbook_title": textbook_title,
                        "textbook_description": textbook_description,
                        "chapter_title": chapter_title,
                        "section_title": section_title,
                        "type": "section_content",
                        "chapter_number": i + 1,
                        "section_number": j + 1
                    }

                    point_id = qdrant_manager.store_text_chunk(
                        text=section_content,
                        metadata=metadata,
                        vector=embedding
                    )

                    total_processed += 1
                    print(f"Stored section '{section_title}' in chapter '{chapter_title}' with ID: {point_id}")

        # Log successful API call
        log_api_call(
            endpoint="/ingest/textbook-full",
            method="POST",
            params={
                "textbook_title": textbook_data.get("title", "unknown"),
                "chapters_count": len(textbook_data.get("chapters", []))
            },
            response={
                "total_processed": total_processed,
                "total_chapters": total_chapters
            }
        )

        return {
            "status": "success",
            "message": f"Successfully ingested textbook: {textbook_title}",
            "textbook_title": textbook_title,
            "chapters_processed": total_chapters,
            "total_items_processed": total_processed
        }

    except Exception as e:
        # Log the error
        log_error(
            error_msg=str(e),
            context={
                "textbook_title": textbook_data.get("title", "unknown") if 'textbook_data' in locals() else 'unknown'
            }
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while ingesting the full textbook"
        )