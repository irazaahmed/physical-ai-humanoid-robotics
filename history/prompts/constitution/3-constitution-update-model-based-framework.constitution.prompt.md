---
id: 3
title: constitution-update-model-based-framework
stage: constitution
date: 2025-12-13
surface: agent
model: Qwen
feature: physical-ai-humanoid-robotics
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "model-based", "framework"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - .specify/memory/constitution.md validates against requirements
---

## Prompt

```
/sp.constitution You are working inside a Spec-Kit Plus based hackathon repository.

Spec-Kit Plus history tracking MUST remain enabled.
All changes must be traceable in prompt history.

====================================
FILE TO UPDATE (IMPORTANT)
====================================

Update ONLY this file:
Physical AI & Humanoid Robotics Hackathon Constitution

Do NOT rename the file.
Do NOT move the file.
Do NOT delete any existing sections.

This is an UPDATE, not a rewrite.

====================================
PRIMARY GOAL
====================================

Enhance the constitution by adding a NEW, CLEAR, and DETAILED section
that explains the MODEL-BASED (PROJECT-BASED) evolution of the hackathon work.

This section must clearly explain:
- That the hackathon is intentionally executed in multiple MODELS (Projects)
- That each model has a strict scope
- That requirements are fulfilled progressively, not all at once

====================================
NEW SECTION TO ADD
====================================

Add a NEW top-level section titled:

## Textbook & System Evolution Model (Model-Based Hackathon Framework)

This section must be added AFTER:
“Project Layout and Structure”
and BEFORE:
“Scoring-Aligned Success Criteria”

====================================
WHAT THIS NEW SECTION MUST EXPLAIN
====================================

The section must clearly and explicitly define FOUR MODELS:

------------------------------------
MODEL 1 — Core AI-Native Textbook
------------------------------------

Purpose:
- Establish the foundation of the Physical AI & Humanoid Robotics textbook

Includes:
- Docusaurus setup (TypeScript)
- Core book structure
- Initial modules and chapters
- Spec-Kit Plus driven content generation
- Claude Code assisted writing
- No chatbot
- No backend
- No authentication
- No personalization
- No translation

Outcome:
- A complete, readable, structured AI-native textbook foundation

------------------------------------
MODEL 2 — Textbook Refinement & Expansion
------------------------------------

Purpose:
- Improve and evolve the textbook created in Model 1

Includes:
- Refined chapter templates
- Consistent pedagogy across modules
- Expanded explanations, labs, exercises
- Improved learning outcomes
- Structural consistency and navigation
- Content depth and clarity improvements

Explicitly Excludes:
- RAG chatbot
- FastAPI backend
- Databases
- Authentication
- Personalization
- Translation

Outcome:
- A mature, high-quality, production-ready textbook

------------------------------------
MODEL 3 — RAG Chatbot & Intelligent Interaction
------------------------------------

Purpose:
- Add intelligent interaction on top of the completed textbook

Includes:
- RAG chatbot
- Selection-based Question Answering
- FastAPI backend
- Embedding pipeline
- Vector databases (Neon + Qdrant)
- OpenAI Agents / ChatKit integration
- Hallucination mitigation
- API endpoints
- Frontend chatbot integration

Depends On:
- Model 1
- Model 2

Outcome:
- An AI-powered interactive textbook experience

------------------------------------
MODEL 4 — Advanced Features & Bonus Scoring
------------------------------------

Purpose:
- Achieve bonus points and advanced personalization

Includes:
- Better-Auth signup/signin
- User profiling
- Content personalization
- Urdu translation (RTL support)
- Reusable intelligence
- Subagents
- Learning analytics
- Demo polish and submission readiness

Outcome:
- Fully featured, high-scoring hackathon submission

====================================
SCORING CLARITY (VERY IMPORTANT)
====================================

In this section, clearly state:

- Base 100 points can be achieved by completing:
  Model 1 + Model 2 + Model 3

- Bonus points are primarily achieved in:
  Model 4

- Partial completion of models is allowed during development
  as long as final submission aligns with scoring criteria

====================================
ALIGN WITH EXISTING SECTIONS
====================================

While adding this section:

- Do NOT contradict existing scoring rules
- Do NOT remove any technical requirements
- Treat existing sections as GLOBAL requirements
- Clarify that GLOBAL requirements are fulfilled progressively per model

====================================
STYLE & WRITING RULES
====================================

- Professional, authoritative tone
- Clear headings and sub-headings
- No implementation code
- No ambiguity
- No future guessing
- Clear separation of concerns per model

====================================
FINAL INSTRUCTION
====================================

- Insert the new section cleanly into the constitution
- Keep all existing content intact
- Update version number appropriately (e.g. 1.2.0 → 1.3.0)
- Update SYNC IMPACT REPORT to reflect:
  “Added Model-Based Hackathon Framework section”
- End after saving the updated constitution

END
```

## Response snapshot

Successfully updated the Physical AI & Humanoid Robotics Hackathon Constitution by adding the "Textbook & System Evolution Model (Model-Based Hackathon Framework)" section as requested. The new section clearly defines the four models (Model 1-4) with their specific purposes, inclusions, exclusions, and outcomes. The section also clarifies the scoring alignment, stating that Base 100 points can be achieved by completing Models 1-3, with bonus points primarily from Model 4. The version number was updated from 1.2.0 to 1.3.0, and the sync impact report was updated to reflect the addition of the Model-Based Hackathon Framework section.

## Outcome

- ✅ Impact: Successfully added the Model-Based Hackathon Framework section to provide clear guidance on the progressive development approach
- 🧪 Tests: Updated constitution validates against requirements
- 📁 Files: Modified .specify/memory/constitution.md with the new section
- 🔁 Next prompts: Constitution now provides clearer project direction for future development
- 🧠 Reflection: The addition clarifies the multi-model approach which will help with project organization and milestone tracking

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): Standard /sp.constitution command with model-based framework requirements
- Next experiment (smallest change to try): Implement the model-based approach in project planning