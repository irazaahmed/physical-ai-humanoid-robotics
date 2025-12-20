<!-- SYNC IMPACT REPORT
Version change: 1.3.0 -> 1.4.0
List of modified principles: Enhanced existing sections and added new requirement sections
Added sections: Selection based QA specifications, Detailed scoring mapping, Deployment requirements, Embedding pipeline specifications, User data and privacy requirements, Testing requirements, Updated template update requirements, Hackathon Execution Model (Book & System Evolution) with Model 1–4 breakdown
Removed sections: Textbook & System Evolution Model (Model-Based Hackathon Framework)
Templates requiring updates: ✅ .specify/templates/plan-template.md (to include QA behavior requirements) / ✅ .specify/templates/spec-template.md (to include scoring alignment requirements) / ✅ .specify/templates/tasks-template.md (to include testing requirements)
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Hackathon Constitution

## Mission Statement

This AI-native textbook and RAG chatbot system exists to democratize advanced education in Physical AI and humanoid robotics. By combining AI-assisted content generation with interactive learning experiences, we aim to accelerate the development of skilled practitioners who can contribute meaningfully to the rapidly evolving field of humanoid robotics. This project serves as both a cutting-edge educational tool and a demonstration of AI-assisted learning methodologies.

## Project Scope

**INCLUDED IN SCOPE:**
- AI-native textbook creation using Docusaurus + Spec-Kit Plus + Claude Code
- Retrieval-Augmented Generation (RAG) chatbot with FastAPI backend
- Vector database integration (Neon Serverless Postgres, Qdrant Cloud)
- Isaac Sim GPU-accelerated simulation environment
- ROS2 and Gazebo integration for robot control
- Support for NVIDIA Jetson development kits
- OpenAI Agents and ChatKit integration
- GitHub Pages or Vercel deployment
- Subagent architectures for complex reasoning
- Better-Auth implementation for user authentication
- Personalization features for adaptive learning
- Urdu translation support for multilingual accessibility
- Demo and submission preparation for hackathon evaluation

**EXCLUDED FROM SCOPE:**
- Full-scale physical robot manufacturing
- Real-time hardware control beyond simulation
- Commercial licensing of proprietary robot platforms
- Deep reinforcement learning training beyond curriculum exercises
- Advanced computer vision beyond curriculum needs
- Custom neural network development beyond provided tools
- Third-party hardware integration beyond specified platforms

## Core Deliverables

### 1. AI-Native Textbook System
The Docusaurus-based textbook system with AI-assisted content generation capabilities. All content must be generated using Spec-Kit Plus workflows with Claude Code integration.

### 2. RAG Chatbot System
A FastAPI-based retrieval-augmented generation system with vector database storage. The system must support user-selected text question answering with contextual responses.

### 3. Integration Logic
Backend services connecting the textbook content with the RAG system, enabling seamless information retrieval and user interaction flows.

### 4. Demo and Submission Materials
Comprehensive demonstration materials, technical documentation, and submission artifacts for hackathon evaluation.

## Project Layout and Structure

### Docusaurus Root Folder Convention
The root folder for the textbook must be named 'docusaurus' and must remain unchanged across all phases of the project.

The Docusaurus root folder name is permanently 'docusaurus'. It cannot be renamed. All documentation, examples, paths, sidebars, build commands, CI scripts, and future automation flows must follow this exact folder structure.

## Hackathon Execution Model (Book & System Evolution)

The Physical AI & Humanoid Robotics Hackathon follows a Model-Based approach where work is executed in progressive models, each with specific scope and requirements. This framework enables systematic development and ensures that complex systems are built in manageable, testable increments.

### MODEL 1 — Core AI-Native Textbook

**Purpose:**
Create the complete Physical AI & Humanoid Robotics textbook.

**Includes:**
- Docusaurus-based textbook (TypeScript)
- AI-native content generation using Spec-Kit Plus + Claude Code
- Full course coverage:
  - Module 1: ROS 2 (Robotic Nervous System)
  - Module 2: Gazebo & Unity (Digital Twin)
  - Module 3: NVIDIA Isaac (AI-Robot Brain)
  - Module 4: Vision-Language-Action (VLA)
- Theory, labs, exercises, examples, learning outcomes
- Navigation, structure, and readability

**Explicitly Excludes:**
- RAG chatbot
- Backend services
- Authentication
- Personalization
- Urdu translation
- Subagents

**Outcome:**
A complete, static, AI-native textbook.

### MODEL 2 — Intelligent Textbook (RAG Integration)

**Purpose:**
Add intelligent interaction to the completed textbook.

**Includes:**
- Integrated RAG chatbot
- Selection-based Question Answering
- FastAPI backend
- OpenAI Agents / ChatKit SDK
- Neon Serverless Postgres
- Qdrant Cloud (free tier)
- Embedding and retrieval pipeline
- Hallucination mitigation
- Chatbot embedded inside the book UI

**Depends On:**
Model 1

**Explicitly Excludes:**
- Authentication
- Personalization
- Urdu translation
- Subagents

**Outcome:**
An interactive textbook capable of answering questions
about its own content, including selected text.

### MODEL 3 — Reusable Intelligence & Authentication

**Purpose:**
Achieve bonus points through system intelligence and user identity.

**Includes:**
- Claude Code Subagents
- Reusable intelligence components
- Better-Auth signup and signin
- User background data collection
- Secure session handling

**Depends On:**
Model 1 + Model 2

**Outcome:**
A smarter system with reusable AI logic and authenticated users.

### MODEL 4 — Personalization & Localization

**Purpose:**
Maximize bonus scoring and learning experience.

**Includes:**
- Chapter-level personalization button
- Content adaptation based on user background
- Urdu translation toggle
- RTL layout support
- Enhanced UX polish

**Depends On:**
Model 1 + Model 2 + Model 3

**Outcome:**
A fully personalized, multilingual AI-native textbook.

### Scoring Alignment

- **Base 100 points** require completion of: Model 1 + Model 2
- **Bonus points** are earned through: Model 3 and Model 4
- Models are intentionally executed step-by-step
  and not all requirements are expected in early models.

All existing technical requirements and specifications remain globally applicable, but are fulfilled progressively according to the model-based framework.

## Scoring-Aligned Success Criteria

### Base 100 Points Criteria:
- Complete AI-native textbook with educational content on Physical AI and humanoid robotics
- Functional RAG chatbot that accurately responds to user queries about textbook content
- Proper integration between textbook and chatbot systems
- Successful deployment to GitHub Pages or Vercel
- Demonstration of textbook content with Isaac Sim simulation
- Use of specified technology stack (Docusaurus, FastAPI, Neon, Qdrant, OpenAI SDKs)
- Clear documentation of architecture and implementation
- Evidence of Claude Code usage in development process

### Bonus Points (Up to +150 Points):
- **Subagents**: Implementation of multi-agent systems for complex reasoning tasks (+25 points)
- **Reusable Intelligence**: Creation of modular AI components that can be repurposed across different contexts (+25 points)
- **Better-Auth Signup Flow**: Secure user authentication and authorization system (+25 points)
- **Personalization**: Adaptive learning features based on user behavior and preferences (+25 points)
- **Urdu Translation**: Full or partial translation of textbook content to Urdu language (+25 points)
- **Additional Bonus Features**: Up to 25 extra points for exceptional implementations exceeding baseline expectations

## Selection-Based Question Answering Specification

### User Interaction Behavior
- When users select text within the textbook, a contextual question input field appears
- Users can ask questions specifically about the selected text portion
- The system must provide responses directly related to the selected content and its context
- The chatbot interface must be seamlessly integrated into the textbook experience

### Fallback Rules
- If selected text cannot be found in the vector store: fallback to general textbook content search
- If no relevant information is found: provide a response acknowledging the lack of relevant content
- If user query is ambiguous: request clarification from the user before attempting to answer
- In case of API failures: display an appropriate error message and suggest retrying

### Acceptance Test
- Given a user has selected a text segment about humanoid robot kinematics
- When the user asks "Explain the forward kinematics equation mentioned here?"
- Then the system must return a response that directly references and explains the specific equation in the selected text
- And the response must be accurate and relevant to the selected context

## Detailed Scoring to Acceptance Criteria Mapping

### Base 100 Points Implementation:
- **Complete textbook content**: Minimum 10 chapters with 3000+ words each covering Physical AI concepts
- **RAG functionality**: User can select text and get accurate answers (90%+ accuracy on test questions)
- **System integration**: Seamless connection between textbook UI and chatbot backend
- **Deployment**: Live, accessible site on GitHub Pages or Vercel with all features functional
- **Isaac Sim integration**: Demonstrable simulation examples linked to textbook concepts
- **Technology stack compliance**: Using all required frameworks as specified
- **Documentation**: Complete architecture diagram, API documentation, and setup guides
- **Claude Code usage**: Evidenced in commit history and development artifacts

### Bonus Points Implementation:
- **Subagents (+25)**: Implementation of at least 2 specialized agents that handle different types of queries
- **Reusable Intelligence (+25)**: At least 3 modular AI components that can be repurposed in other projects
- **Better-Auth (+25)**: Complete authentication flow with login, signup, and user profile management
- **Personalization (+25)**: User learning progress tracking with adaptive content recommendations
- **Urdu Translation (+25)**: Minimum 50% of textbook content translated to Urdu with proper RTL layout

## Technical Constraints

### Framework Requirements:
- **Frontend**: Docusaurus for static site generation with React-based customization
- **Backend**: FastAPI for RAG service with async processing capabilities
- **Databases**: Neon Serverless Postgres for relational data, Qdrant Cloud for vector embeddings
- **AI Services**: OpenAI SDKs, Agents framework, and ChatKit integration
- **Spec-Kit Plus**: Claude Code workflows for AI-assisted development

### Hardware Simulation Constraints:
- **Primary Platform**: Isaac Sim GPU-accelerated simulation environment
- **Robot Models**: ROS2 and Gazebo-compatible humanoid robots
- **Development Kits**: NVIDIA Jetson series for edge computing demonstrations
- **Cloud Access**: GPU resources for heavy computational tasks

### Deployment Constraints:
- **Hosting**: GitHub Pages or Vercel for textbook content
- **Backend**: Cloud hosting for RAG services with scalable infrastructure
- **Database**: Neon Serverless Postgres and Qdrant Cloud for vector storage

### Time Constraints:
- **Hackathon Duration**: 4-week intensive development cycle
- **Weekly Milestones**: Progress checkpoints for iterative development
- **Final Submission**: Complete deliverables by hackathon deadline

### Cloud vs Local Simulation:
- **Local Simulation**: Isaac Sim and Gazebo for development and testing
- **Cloud Processing**: Heavy computation and model serving when local resources insufficient
- **Real Robot Control**: Simulated environment with potential for limited hardware access

## Deployment Requirements

### CI Pipeline Expectations for Docusaurus:
- Automated build verification on every commit
- Automated deployment to GitHub Pages or Vercel on merge to main branch
- Linting and formatting checks before build
- Unit tests for any custom React components
- Accessibility checks (a11y) as part of the pipeline
- Performance budget verification to ensure fast loading

### Backend Tests:
- FastAPI endpoints must have 90%+ code coverage
- Integration tests for the RAG functionality
- Load testing to ensure system stability under concurrent users
- Database connection and query tests
- Embedding generation and retrieval tests
- Authentication flow tests (if Better-Auth is implemented)

## Embedding Pipeline Specification

### Chunk Size and Overlap:
- **Chunk Size**: 512 tokens for optimal retrieval accuracy
- **Overlap**: 64 tokens to preserve context across chunks
- **Minimum Chunk Size**: 128 tokens to ensure meaningful content
- **Maximum Chunk Size**: 1024 tokens to balance context and precision

### Embedding Model Family:
- **Primary Model**: OpenAI text-embedding-3-small for cost efficiency
- **Alternative Model**: text-embedding-3-large for higher quality when needed
- **Backup Model**: Compatible models from HuggingFace if OpenAI unavailable

### Qdrant Collection Naming Rules:
- **Format**: `{book_title}_{chapter_number}_{section_type}` (e.g., `physical_ai_03_hardware_introduction`)
- **Metadata Schema**: Include page number, chapter title, section title, and tags
- **Vector Configuration**: 1536 dimensions to match embedding model output
- **Payload Indexing**: Enable indexing on all metadata fields for efficient filtering

## User Data, Privacy, and Better-Auth Requirements

### Data Handling:
- All user selections and queries are logged for improving the system
- Personal information is only stored with explicit user consent
- No sensitive personal data (SSN, financial info) is collected or stored
- User authentication data is encrypted at rest and in transit

### Privacy Policy:
- Users have rights to view, modify, or delete their data
- Data retention period: 2 years from last user activity
- Data backups are encrypted and stored securely
- User data is never sold or shared with third parties

### Better-Auth Implementation:
- Support for email/password and OAuth (Google, GitHub) login
- Passwords must meet complexity requirements
- Session management with configurable timeout
- Account verification via email confirmation
- Password reset functionality with secure tokens

### Demo User Retention Policy:
- Demo accounts are automatically deleted after 30 days of inactivity
- Demo usage data is anonymized after 90 days
- Demo user data is excluded from model training processes
- Demo accounts have limited access to certain features (e.g., export capabilities)

## Testing Requirements

### FastAPI Unit Tests:
- Minimum 90% code coverage for all API endpoints
- Test validation of request/response schemas
- Test error handling and edge cases
- Test database connection and query functions
- Mock external services (OpenAI APIs, etc.) during testing

### End-to-End Test for Selection-Based QA:
- Simulate user selecting text in the textbook UI
- Submit a question related to the selected text
- Verify the response is contextually relevant and accurate
- Test various selection sizes and content types
- Test fallback mechanisms when no relevant content is found

### Smoke Tests for Docusaurus Build:
- Verify all pages load without JavaScript errors
- Check that all internal links are valid
- Validate that the RAG chatbot component loads properly
- Confirm that search functionality works
- Test that code blocks render properly
- Verify mobile responsiveness of all components

## High-Level Architecture

### Book System Architecture:
- **Content Layer**: Markdown and MDX documents in Docusaurus structure
- **AI Assistant Layer**: Claude Code integration for content generation and refinement
- **Spec-Kit Plus Pipeline**: Automated workflows for specification-driven development
- **Static Site Generation**: Docusaurus compilation to optimized static assets

### RAG Backend Architecture:
- **Document Ingestion Pipeline**: Automated extraction and embedding of textbook content
- **Vector Database**: Neon Postgres for metadata and Qdrant Cloud for embeddings
- **Query Processing Service**: FastAPI endpoints for semantic search and response generation
- **Integration Layer**: Connection between textbook frontend and RAG backend

### User Interaction Flows:
- **Content Exploration**: Users browse textbook content through Docusaurus interface
- **Question Submission**: Users select text portions and submit questions via chatbot
- **Response Generation**: RAG system retrieves relevant content and generates contextual answers
- **Learning Analytics**: Tracking of user engagement and comprehension metrics

## User Personas

### Students
Educators and learners seeking advanced knowledge in Physical AI and humanoid robotics. Need accessible content with practical examples and interactive tools to reinforce learning concepts.

### Authors
Subject matter experts creating educational content using AI-assisted tools. Require efficient workflows that leverage Claude Code capabilities while maintaining content accuracy and pedagogical effectiveness.

### Panaversity Reviewers
Hackathon evaluators assessing submissions for technical merit, educational value, and innovation. Need clear documentation and demonstration of implemented features.

### AI Agents/Subagents
Claude Code and other AI systems assisting in content generation, code implementation, and system integration. Require well-structured specifications and clear interfaces.

## Governance & Workflow Rules

### Version Control
- All code and content stored in Git repository with detailed commit messages
- Branch-based development with pull requests for all changes
- Regular backup and synchronization across team members

### AI-Assisted Editing Boundaries
- Claude Code must be used for substantial content generation and refinement
- Human oversight required for all technical and educational content accuracy
- Clear attribution of AI assistance in development process documentation

### Required Review Cycles
- Weekly internal reviews of progress and direction
- Bi-weekly external reviews with domain experts
- Final comprehensive review before submission

### Accessibility and Multilingual Expectations
- WCAG 2.1 AA compliance for web accessibility
- Urdu translation as primary localization requirement
- Clear navigation and responsive design for diverse user needs

## Risks & Mitigations

### GPU Hardware Limitations
Risk: Insufficient GPU resources for complex simulations during development.
Mitigation: Leverage cloud GPU services and optimize simulation complexity for available hardware.

### Cloud Latency for Real-Robot Control
Risk: Network latency affecting real-time robot control demonstrations.
Mitigation: Implement prediction algorithms and local control buffers, focus on simulation for demo.

### LLM Hallucination and Mitigation
Risk: AI-generated content containing factual inaccuracies.
Mitigation: Implement rigorous fact-checking workflows and citation requirements for all content.

### Content Accuracy Validation
Risk: Educational content containing errors that mislead students.
Mitigation: Multi-tier review process with domain experts and automated correctness checks.

## Licensing & Publication Expectations

The textbook and associated systems will be published as open educational resources under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. This enables broad community usage while protecting commercial interests. Code components will use standard open-source licenses appropriate to their functions. GitHub publication ensures wide accessibility and community contribution opportunities.

## Template Update Requirements

### plan-template.md Updates:
- Add sections for RAG architecture planning
- Include evaluation criteria for selection-based QA functionality
- Add risk assessment related to vector database implementation
- Include timeline for both textbook content creation and backend development

### spec-template.md Updates:
- Add detailed API specifications for the RAG backend
- Include requirements for embedding pipeline
- Define acceptance criteria for the selection-based QA feature
- Specify integration points between Docusaurus and FastAPI

### tasks-template.md Updates:
- Add tasks related to testing requirements (unit, integration, end-to-end)
- Include tasks for deployment pipeline setup
- Add tasks for embedding pipeline implementation
- Define tasks for Better-Auth integration if included
- Add tasks for demo preparation and submission materials

**Version**: 1.4.0 | **Ratified**: 2025-01-10 | **Last Amended**: 2025-12-13

Update: Enforced permanent Docusaurus root folder naming convention ('docusaurus').
Update: Added Hackathon Execution Model (Book & System Evolution) section with Model 1-4 breakdown.