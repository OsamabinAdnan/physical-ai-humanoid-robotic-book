<!-- Sync Impact Report:
Version change: 1.0.0 → 1.0.1
List of modified principles: All principles expanded for detail.
Added sections: Governance details (Amendment Procedure, Versioning Policy, Compliance Review Expectations).
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/sp.constitution.md: ⚠ pending
Follow-up TODOs: None
-->
# Physical AI Humanoid Robotics - The Future is Now Constitution

## Core Principles

### I. Spec-Driven Structure
**Ensures every chapter originates from a clear, validated specification.**
*   **Explanation**: Every piece of content, from individual sections to full chapters, must begin with a formal specification (`spec.md`). This specification details the content's purpose, scope, target audience, learning objectives, and acceptance criteria. This approach guarantees that all written material is intentionally designed to meet specific educational and informational goals before development begins.

### II. Zero-Hallucination Rule
**Prevents incorrect content by enforcing source-verified information only.**
*   **Explanation**: All factual statements, claims, and data presented in the book must be directly verifiable against authoritative, credible sources. Content not explicitly backed by a referenced source will not be included. This principle is critical for maintaining the scientific integrity and trustworthiness of the publication, especially in rapidly evolving fields like AI and robotics.

### III. Technical Accuracy Enforcement
**Requires all robotics/AI claims to be backed by authoritative references.**
*   **Explanation**: Specifically for technical concepts in robotics and AI, all descriptions, explanations, and assertions must be supported by peer-reviewed academic papers, industry standards, or recognized expert publications. This ensures the technical rigor and correctness of the material, providing readers with reliable, up-to-date information.

### IV. Pedagogical Architecture
**Aligns content with Bloom’s taxonomy and structured learning progression.**
*   **Explanation**: Content is designed following pedagogical best practices, specifically aligning with Bloom's Taxonomy. Chapters and modules are structured to guide learners from foundational knowledge (remembering, understanding) through application, analysis, evaluation, and creation. This ensures a logical and effective learning path, maximizing comprehension and skill development.

### V. Engineering Reproducibility
**Mandates runnable code, verifiable diagrams, and testable examples.**
*   **Explanation**: All code examples provided in the book must be fully runnable and tested. Diagrams should be generated from verifiable data or code where applicable, and all presented examples must be testable to demonstrate their functionality and correctness. This principle promotes active learning and allows readers to experiment and confirm concepts independently.

### VI. Consistency of Terminology
**Ensures uniform vocabulary across the entire textbook.**
*   **Explanation**: A strict glossary and style guide must be followed to maintain consistent terminology throughout the entire book. Key terms in AI, robotics, and related fields will be defined once and used uniformly to avoid confusion and enhance clarity for the reader.

### VII. Chunkable, RAG-Ready Content
**Formats all text for embeddings, chatbot retrieval, and searchability.**
*   **Explanation**: Content is structured in small, logically coherent chunks that are optimized for embedding, retrieval-augmented generation (RAG) systems, and efficient search. This facilitates integration with AI-powered learning tools and allows readers to quickly find specific information through advanced search capabilities.

### VIII. Reading-Level Control
**Maintains clear writing within Flesch-Kincaid grade 10–12 for accessibility.**
*   **Explanation**: All prose must adhere to a Flesch-Kincaid grade level between 10 and 12. This ensures that the material is accessible to an academic audience with a computer science background without oversimplifying complex technical topics. Regular checks will be performed to maintain this standard.

### IX. Curriculum Alignment
**Forces strict adherence to the 4-module Physical AI course structure.**
*   **Explanation**: The book's content, structure, and depth must strictly align with the established 4-module Physical AI course curriculum. Each chapter and section must map directly to specific learning objectives within this curriculum, ensuring comprehensive coverage and direct applicability for students following the course.

### X. Style & Quality Standards
**Enforces clarity, coherence, and precision across all chapters.**
*   **Explanation**: Beyond technical accuracy, all content must meet high standards of clarity, coherence, and precision in writing style. This includes adherence to grammar, syntax, logical flow, and overall readability, ensuring a professional and engaging reading experience.

### XI. Citation Integrity
**Prohibits hallucinated sources and requires validated academic documentation.**
*   **Explanation**: All citations must refer to actual, verifiable academic or authoritative sources. The creation or inclusion of non-existent or fabricated sources (hallucinations) is strictly prohibited. A robust verification process will be in place to confirm the existence and relevance of all cited materials.

### XII. Interactivity Requirements
**Makes content explanation-friendly, searchable, and chatbot-compatible.**
*   **Explanation**: Content is designed to be highly interactive, enabling easy explanation by AI tutors, efficient searchability by users, and seamless integration with conversational AI interfaces (chatbots). This includes using clear headings, concise paragraphs, and structured data where appropriate. **CRITICAL NOTE**: Interactivity features (RAG, chatbots, personalization, authentication) are secondary to core content creation and must only be implemented after all content requirements are satisfied.

### XIII. Governance & Validation
**Applies continuous self-checking to ensure correctness and compliance.**
*   **Explanation**: The entire content development process is subject to continuous self-checking mechanisms to ensure ongoing correctness and compliance with all constitutional principles. This involves automated checks, peer reviews, and regular audits of content against its specifications and the principles outlined herein.

### XIV. Deployment Guarantee
**Ensures content builds cleanly in Docusaurus and integrates with the RAG backend.**
*   **Explanation**: All content must be compatible with and build cleanly within the Docusaurus framework, which serves as the publishing platform. Furthermore, the content must seamlessly integrate with the Retrieval-Augmented Generation (RAG) backend, ensuring it is ready for advanced AI-driven information retrieval and interaction.

## Governance
This section outlines the procedures for managing and evolving this constitution, ensuring its ongoing relevance and effectiveness.

### Amendment Procedure
*   **Proposal**: Any proposed amendment to this constitution must be formally documented, detailing the rationale for the change, the specific sections affected, and the anticipated impact.
*   **Review**: Proposed amendments will undergo a thorough review by a designated committee or core project team to assess alignment with project goals, potential consequences, and consensus.
*   **Approval**: Amendments require explicit approval from the project lead or a majority consensus of the core project team before implementation.
*   **Documentation**: All approved amendments must be recorded with their effective date, the version number, and a summary of changes in the project's changelog or a dedicated `ADR`.

### Versioning Policy
This constitution adheres to Semantic Versioning (SemVer) principles: **MAJOR.MINOR.PATCH**.
*   **MAJOR Version Increment**: Reserved for backward-incompatible changes, such as the removal or fundamental redefinition of core principles or governance rules.
*   **MINOR Version Increment**: Applied when new principles, sections, or materially expanded guidance are added without breaking existing adherence requirements.
*   **PATCH Version Increment**: Used for clarifications, wording refinements, typo corrections, and other non-semantic adjustments that do not alter the meaning or impact of existing rules.

### Compliance Review Expectations
*   **Regular Audits**: The project lead or designated quality assurance personnel will conduct regular audits of content and development practices to ensure continuous compliance with all constitutional principles.
*   **Code/Content Reviews**: During pull requests or content submission reviews, reviewers are mandated to verify adherence to relevant constitutional principles.
*   **Justification for Complexity**: Any deviation from principles aimed at simplicity or directness must be explicitly justified and documented, outlining the necessity and mitigation strategies.
*   **Guidance Document**: Refer to `CLAUDE.md` for specific runtime development guidance and operational procedures that complement this constitution.

**Version**: 1.0.1 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05