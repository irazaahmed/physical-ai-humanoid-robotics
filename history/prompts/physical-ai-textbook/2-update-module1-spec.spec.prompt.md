---
id: 2
title: update-module1-spec
stage: spec
date: 2025-12-12
model: Claude Sonnet 4
feature: physical-ai-textbook
branch: 1-physical-ai-textbook
user: Assistant
command: /sp.specify
labels: ["specification", "textbook", "ai", "robotics", "module1", "ros2"]
links:
  spec: specs/1-physical-ai-textbook/spec.md
  checklist: specs/1-physical-ai-textbook/checklists/requirements.md
files:
  - specs/1-physical-ai-textbook/spec.md
tests: []
outcome: Specification updated to focus on Project 1 Module 1 only
evaluation: Specification now properly scoped to Module 1 textbook content only
---

# Feature Specification Update: Physical AI, Project 1 — Module 1: Robotic Nervous System (ROS 2)

## Original User Prompt

Update the existing spec.md file in-place, do not rename it, do not move it, preserve file history. Replace or revise the content so that the spec focuses only on Project 1, "Book: Module 1, Robotic Nervous System (ROS 2)". Keep the rest of the repository and global constitution referenced, but limit this spec to Project 1 scope only. Requirements for the update, follow exactly: 1) Metadata header, add or update front matter: - project_id: project-1 - title: "Physical AI, Project 1 — Module 1: Robotic Nervous System (ROS 2)" - version: 1.0.0 - author: Ahmed Raza - date: <today's date in YYYY-MM-DD format> - related_constitution_ref: constitution.md 2) Project Objective, one short paragraph: - Create Module 1 textbook content using Spec-Kit Plus and Claude Code, integrate content into Docusaurus template already in repo, produce deployable Docusaurus docs for module1, no RAG or backend work in this project. 3) In-scope, list explicitly: - Create the following markdown files under docusaurus/docs/module1/: - intro.md - ros-overview.md - nodes-topics-services.md - rclpy-basics.md - urdf-intro.md - lab1-pubsub.md - lab2-controller.md - quiz.md - references.md - appendix-hardware-lite.md - Each generated markdown must include JSON frontmatter at top with keys: doc_id, title, module, estimated_tokens. Use doc_id pattern: project1_m1_ch01, project1_m1_ch02, ... - Update or add a sidebar entry snippet for Module 1 in the spec, pointing to docusaurus/docs/module1/* files, so downstream automation can update sidebars.js. - Add per-doc metadata placeholders for future RAG fields, keys: embedding_required: true, chunk_hint_tokens: 500 4) Out-of-scope, declare clearly: - No RAG backend implementation, no FastAPI, no Qdrant, no Neon, no OpenAI/Claude agent wiring for this project. - No Better-Auth, no personalization, no Urdu translation, no subagents, no Isaac Sim heavy demos. Only conceptual notes about simulation allowed. 5) Deliverables, precise files and checks: - docusaurus/docs/module1/*.md as listed above - spec.md updated in-place - README.md short section: commands to build and preview Docusaurus locally - Acceptance checklist block inside spec.md listing exact pass/fail checks 6) Acceptance criteria, concrete and testable: - Local render: running `cd docusaurus && npm run start` must show Module 1 pages, no build errors. - Build: `npm run build` must succeed, `npm run serve` must serve static files. - Chapter content: every chapter must include - Objective (1-2 lines) - 2-3 Learning outcomes - At least one hands-on lab with step-by-step commands - At least one code snippet, <= 20 lines for examples - One or more exercises or quiz questions - Sidebar: Module 1 appears in sidebars in correct order - Frontmatter: each doc contains the JSON frontmatter keys required - File size: each chapter < 3000 words - README: contains build, preview, and deploy hint for GitHub Pages or Vercel 7) Machine friendly instructions for downstream automation: - Provide a short "doc generation map" table inside spec.md mapping doc_id to target path and a short content brief of 1 sentence per doc. - Include explicit doc_id naming rules, example: project1_m1_ch01 = intro.md - Include embedding_placeholder metadata lines so later RAG pipeline can detect documents to index 8) Risks and mitigations, two short bullets: - Risk: Claude Code may generate technical inaccuracies, Mitigation: add human reviewer step in next plan to verify labs and code - Risk: Docusaurus render errors for MDX, Mitigation: constrain content to plain markdown compatible with Docusaurus default 9) Acceptance sign-off - Add a final section "Sign-off" with role placeholders: writer, dev, reviewer, with checkboxes for acceptance tests 10) Important constraints and final instructions: - Do not alter file name spec.md, do not change file location - Preserve existing git/spec history comments if present, add a small changelog entry at top of spec.md noting this update, who requested it, and date - Keep language concise, machine parsable, use markdown headings, lists, and a small JSON table for doc mapping - End the file with a "next step" line that tells automation to run qwen CLI prompt to generate plans/project-1-plan.md next Perform the update now, overwrite content of spec.md with the Project 1 focused specification, and return the updated file content as the tool output.

## Specification Updated

The specification has been updated in-place at specs/1-physical-ai-textbook/spec.md to focus exclusively on Project 1: Module 1, Robotic Nervous System (ROS 2). All references to other modules, RAG chatbot, backend services, and out-of-scope features have been removed.

## Key Changes

- Updated frontmatter with project-specific metadata
- Added changelog entry noting the update
- Focused content exclusively on Module 1 (ROS 2 fundamentals)
- Defined 10 specific markdown files to be created under docusaurus/docs/module1/
- Added doc generation map for automation
- Included frontmatter and metadata requirements for future RAG pipeline
- Limited scope to textbook content creation only, excluding RAG and backend work
- Added specific acceptance criteria for Docusaurus integration
- Included next step instructions for automation

## Status

The specification is updated and ready for the next phase of the project (planning).