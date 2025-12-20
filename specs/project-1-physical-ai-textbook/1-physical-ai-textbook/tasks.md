---
tasks_id: project-1-tasks
project_ref: project-1
version: 1.0.0
author: Ahmed Raza
date: 2025-12-12
based_on_plan: plan.md
---

# Tasks: Physical AI Project 1 - Module 1: Robotic Nervous System (ROS 2)

## Global Convention
The root folder for the textbook must be named 'docusaurus' and must remain unchanged across all phases of the project.

The Docusaurus root folder name is permanently 'docusaurus'. It cannot be renamed. All documentation, examples, paths, sidebars, build commands, CI scripts, and future automation flows must follow this exact folder structure.

## Phase 1: Setup Tasks

### Initialize docusaurus Template
- [ ] T1.1 Initialize docusaurus template in docusaurus directory per implementation plan
- [ ] T1.2 Create module1 folder in docusaurus/docs/module1/ per implementation plan
- [ ] T1.3 Add sidebar snippet for Module 1 in docusaurus/sidebars.js per implementation plan

## Phase 2: Foundational Tasks

### Content Creation Setup
- [ ] T2.0 Setup Claude Code and Spec-Kit Plus for content generation per implementation plan

## Phase 3: Chapter Draft Generation (M2)

### [US1] Generate Module 1 Chapter Drafts
- [ ] T2.1 [US1] Generate intro.md with proper frontmatter in docusaurus/docs/module1/intro.md (Objective, Learning outcomes)
- [ ] T2.2 [US1] Generate ros-overview.md with proper frontmatter in docusaurus/docs/module1/ros-overview.md (Objective, Learning outcomes)
- [ ] T2.3 [US1] Generate nodes-topics-services.md with proper frontmatter in docusaurus/docs/module1/nodes-topics-services.md (Objective, Learning outcomes)
- [ ] T2.4 [US1] Generate rclpy-basics.md with proper frontmatter in docusaurus/docs/module1/rclpy-basics.md (Objective, Learning outcomes)
- [ ] T2.5 [US1] Generate urdf-intro.md with proper frontmatter in docusaurus/docs/module1/urdf-intro.md (Objective, Learning outcomes)

## Phase 4: Labs, Quizzes, and References (M3)

### [US2] Create Laboratory Exercises
- [ ] T3.1 [US2] Create lab1-pubsub.md with hands-on lab content in docusaurus/docs/module1/lab1-pubsub.md (Lab instructions, code snippets)
- [ ] T3.2 [US2] Create lab2-controller.md with hands-on lab content in docusaurus/docs/module1/lab2-controller.md (Lab instructions, code snippets)

### [US3] Create Assessment and Reference Materials
- [ ] T3.3 [US3] Create quiz.md with quiz questions in docusaurus/docs/module1/quiz.md (Questions, answers)
- [ ] T3.4 [US3] Create references.md with references in docusaurus/docs/module1/references.md (References, further reading)
- [ ] T3.5 [US3] Create appendix-hardware-lite.md with hardware recommendations in docusaurus/docs/module1/appendix-hardware-lite.md (Hardware specs, setup)

## Phase 5: Testing and Build (M4)

### [US4] Local Testing and Build
- [ ] T4.1 [US4] Run local docusaurus server with `cd docusaurus && npm run start` to test chapter renders
- [ ] T4.2 [US4] Run build and serve with `cd docusaurus && npm run build && npm run serve`
- [ ] T4.3 [US4] Fix any render errors or build issues in docusaurus output
- [ ] T4.4 [US4] Finalize sidebar navigation in docusaurus/sidebars.js for proper ordering

## Phase 6: Acceptance Testing (M5)

### [US5] Final Review and Acceptance
- [ ] T5.1 [US5] Run acceptance checklist from spec.md to verify requirements met
- [ ] T5.2 [US5] Update README.md with build and deployment instructions per implementation plan
- [ ] T5.3 [US5] Perform final content review for technical accuracy and pedagogical quality

## Dependencies

### User Story Completion Order
1. US1 (Chapter Draft Generation) → US2, US3, US4, US5
2. US2 (Labs) → US4, US5
3. US3 (Quizzes/References) → US4, US5
4. US4 (Testing/Build) → US5
5. US5 (Acceptance) - Final completion

### Parallel Execution Opportunities
- T2.1, T2.2, T2.3, T2.4, T2.5 can be executed in parallel [P]
- T3.1, T3.2, T3.3, T3.4, T3.5 can be executed in parallel after US1 completion [P]

## Implementation Strategy

### MVP Scope
- Focus on US1 (basic chapter content) and US4 (basic build) for initial working version
- Ensure docusaurus builds with placeholder content for first MVP
- Add detailed content in subsequent iterations

### Incremental Delivery
1. Complete Phase 1 & 2: docusaurus setup with placeholder content
2. Complete US1: Basic chapter structure with objectives and outcomes
3. Complete US2 & US3: Add labs, quizzes, and references
4. Complete US4: Build and rendering verification
5. Complete US5: Final review and acceptance

## Tasks Table

| task_id | title | role | est_hours | depends_on | output_path | acceptance_checks |
|--------|-------|------|-----------|------------|-------------|-------------------|
| T1.1 | Initialize docusaurus template | dev | 1 | none | docusaurus/ | docusaurus template properly set up |
| T1.2 | Create module1 folder | dev | 0.5 | T1.1 | docusaurus/docs/module1/ | Module 1 directory exists |
| T1.3 | Add sidebar snippet | dev | 0.5 | T1.2 | docusaurus/sidebars.js | Sidebar entry added for Module 1 |
| T2.1 | Generate intro.md | writer | 2 | T1.2 | docusaurus/docs/module1/intro.md | Frontmatter present, chapter renders, objective and outcomes included |
| T2.2 | Generate ros-overview.md | writer | 3 | T1.2 | docusaurus/docs/module1/ros-overview.md | Frontmatter present, chapter renders, objective and outcomes included |
| T2.3 | Generate nodes-topics-services.md | writer | 3 | T1.2 | docusaurus/docs/module1/nodes-topics-services.md | Frontmatter present, chapter renders, objective and outcomes included |
| T2.4 | Generate rclpy-basics.md | writer | 3 | T1.2 | docusaurus/docs/module1/rclpy-basics.md | Frontmatter present, chapter renders, objective and outcomes included |
| T2.5 | Generate urdf-intro.md | writer | 2.5 | T1.2 | docusaurus/docs/module1/urdf-intro.md | Frontmatter present, chapter renders, objective and outcomes included |
| T3.1 | Create lab1-pubsub.md | dev+writer | 4 | T2. | docusaurus/docs/module1/lab1-pubsub.md | Lab instructions correct, code snippets included, chapter renders |
| T3.2 | Create lab2-controller.md | dev+writer | 4 | T2. | docusaurus/docs/module1/lab2-controller.md | Lab instructions correct, code snippets included, chapter renders |
| T3.3 | Create quiz.md | writer | 1.5 | T2. | docusaurus/docs/module1/quiz.md | Quiz questions included, chapter renders |
| T3.4 | Create references.md | writer | 1 | T2. | docusaurus/docs/module1/references.md | References included, chapter renders |
| T3.5 | Create appendix-hardware-lite.md | writer | 1.5 | T2. | docusaurus/docs/module1/appendix-hardware-lite.md | Hardware references included, chapter renders |
| T4.1 | Run local start | dev | 1 | T2., T3. | none | Local server starts, chapters accessible |
| T4.2 | Run build and serve | dev | 1 | T4.1 | docusaurus/build/ | Build succeeds, static site serves correctly |
| T4.3 | Fix render errors | dev | 3 | T4.2 | none | No build errors, all pages render correctly |
| T4.4 | Finalize sidebar | dev | 0.5 | T4.3 | docusaurus/sidebars.js | Sidebar ordered correctly, navigation works |
| T5.1 | Acceptance checklist run | reviewer | 2 | T4. | ACCEPTANCE.md | All spec acceptance criteria met |
| T5.2 | Update readme | dev | 0.5 | T4. | README.md | Build and deployment instructions updated |
| T5.3 | Final content review | reviewer | 2 | T4. | reviews/notes.md | Content reviewed for accuracy and quality |

## Summary

### JSON Summary
```json
{
  "tasks_count": 18,
  "tasks": [
    {"id": "T1.1", "role": "dev", "est_hours": 1, "output": "docusaurus/"},
    {"id": "T1.2", "role": "dev", "est_hours": 0.5, "output": "docusaurus/docs/module1/"},
    {"id": "T1.3", "role": "dev", "est_hours": 0.5, "output": "docusaurus/sidebars.js"},
    {"id": "T2.1", "role": "writer", "est_hours": 2, "output": "docusaurus/docs/module1/intro.md"},
    {"id": "T2.2", "role": "writer", "est_hours": 3, "output": "docusaurus/docs/module1/ros-overview.md"},
    {"id": "T2.3", "role": "writer", "est_hours": 3, "output": "docusaurus/docs/module1/nodes-topics-services.md"},
    {"id": "T2.4", "role": "writer", "est_hours": 3, "output": "docusaurus/docs/module1/rclpy-basics.md"},
    {"id": "T2.5", "role": "writer", "est_hours": 2.5, "output": "docusaurus/docs/module1/urdf-intro.md"},
    {"id": "T3.1", "role": "dev+writer", "est_hours": 4, "output": "docusaurus/docs/module1/lab1-pubsub.md"},
    {"id": "T3.2", "role": "dev+writer", "est_hours": 4, "output": "docusaurus/docs/module1/lab2-controller.md"},
    {"id": "T3.3", "role": "writer", "est_hours": 1.5, "output": "docusaurus/docs/module1/quiz.md"},
    {"id": "T3.4", "role": "writer", "est_hours": 1, "output": "docusaurus/docs/module1/references.md"},
    {"id": "T3.5", "role": "writer", "est_hours": 1.5, "output": "docusaurus/docs/module1/appendix-hardware-lite.md"},
    {"id": "T4.1", "role": "dev", "est_hours": 1, "output": "none"},
    {"id": "T4.2", "role": "dev", "est_hours": 1, "output": "docusaurus/build/"},
    {"id": "T4.3", "role": "dev", "est_hours": 3, "output": "none"},
    {"id": "T4.4", "role": "dev", "est_hours": 0.5, "output": "docusaurus/sidebars.js"},
    {"id": "T5.1", "role": "reviewer", "est_hours": 2, "output": "ACCEPTANCE.md"},
    {"id": "T5.2", "role": "dev", "est_hours": 0.5, "output": "README.md"},
    {"id": "T5.3", "role": "reviewer", "est_hours": 2, "output": "reviews/notes.md"}
  ],
  "total_estimated_hours": 35.5
}
```

### Role Allocation Summary
- **dev**: 7 tasks, 7.5 hours (21% of total effort)
- **writer**: 8 tasks, 17.5 hours (49% of total effort)
- **dev+writer**: 2 tasks, 8 hours (22% of total effort)
- **reviewer**: 2 tasks, 4 hours (8% of total effort)

Update: Enforced permanent Docusaurus root folder naming convention ('docusaurus').

### Automation Notes
- Tasks T2.1-T2.5 and T3.1-T3.5 can potentially be automated using qwen CLI content generation for initial draft creation
- All technical content (labs, code snippets, exercises) requires human review for accuracy
- Review tasks (T5.1, T5.3) require human verification to ensure technical correctness
- Build and deployment tasks (T4.1-T4.4) can be automated with scripts but require human validation of output