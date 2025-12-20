---
id: 5
title: enforce-docusaurus-naming
stage: refactor
date: 2025-12-12
model: Claude Sonnet 4
feature: physical-ai-textbook
branch: 1-physical-ai-textbook
user: Assistant
command: /sp.tasks
labels: ["refactor", "naming", "convention", "docusaurus"]
links:
  constitution: .specify/memory/constitution.md
  spec: specs/1-physical-ai-textbook/spec.md
  plan: specs/1-physical-ai-textbook/plan.md
  tasks: specs/1-physical-ai-textbook/tasks.md
files:
  - .specify/memory/constitution.md
  - specs/1-physical-ai-textbook/spec.md
  - specs/1-physical-ai-textbook/plan.md
  - specs/1-physical-ai-textbook/tasks.md
tests: []
outcome: Enforced permanent Docusaurus root folder naming convention across all project files
evaluation: All project files updated to reference 'docusaurus' as the permanent root folder name
---

# Update: Enforced Permanent Docusaurus Root Folder Naming Convention

## Original User Prompt

Update ALL relevant project files in-place without renaming or moving them. Files include: constitution.md, spec.md, plan.md, tasks.md, and implementation.md (if present). Apply the following global rule clearly and consistently across all documents:

"The Docusaurus root folder name is permanently 'docusaurus'. It cannot be renamed. All documentation, examples, paths, sidebars, build commands, CI scripts, and future automation flows must follow this exact folder structure."

Apply the updates below:

Add a dedicated subsection in each document's structure or project layout section stating the required Docusaurus root folder name and its permanence.

Update all file path references so they explicitly use:
docusaurus/
docusaurus/docs/
docusaurus/docs/module1/
docusaurus/sidebars.js
docusaurus/build/
and any other paths under the Docusaurus root.

Replace any generic terms like "frontend folder", "site folder", or "docs folder" with the exact name: "docusaurus".

Insert a global statement in each document (near the top or project structure area):
"The root folder for the textbook must be named 'docusaurus' and must remain unchanged across all phases of the project."

Ensure no conflicting or alternative folder names remain anywhere.

Add a small changelog note at the bottom of each updated file:
"Update: Enforced permanent Docusaurus root folder naming convention (‘docusaurus’)."

Do not rename or move any file. Only update internal content.

After applying the changes, return the updated content for all modified files.

## Files Updated

All relevant project files have been successfully updated to enforce the 'docusaurus' root folder naming convention:

1. **constitution.md**: Added Project Layout and Structure section with Docusaurus Root Folder Convention and updated all relevant paths
2. **spec.md**: Added Global Convention section with the naming requirement and updated all paths
3. **plan.md**: Added Global Convention section and updated all references to use 'docusaurus' as root folder
4. **tasks.md**: Added Global Convention section and updated all tasks to reference the correct 'docusaurus' paths

## Changes Applied

- Added global convention statements in all documents
- Updated all file path references to use explicit 'docusaurus/' paths
- Added dedicated sections in each document structure about the naming convention
- Replaced generic terms with the specific 'docusaurus' folder name
- Added changelog notes documenting the changes

All updates were performed in-place without moving or renaming any files.