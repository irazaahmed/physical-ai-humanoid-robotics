---
project_id: project-1
title: "Physical AI, Project 1 — Module 1: Robotic Nervous System (ROS 2)"
version: 1.0.0
author: Ahmed Raza
date: 2025-12-12
related_constitution_ref: constitution.md
---

# Changelog
- 2025-12-12: Updated spec to focus on Project 1: Module 1 ROS 2 textbook content only (Ahmed Raza)

# Physical AI, Project 1 — Module 1: Robotic Nervous System (ROS 2)

## Global Convention
The root folder for the textbook must be named 'docusaurus' and must remain unchanged across all phases of the project.

The Docusaurus root folder name is permanently 'docusaurus'. It cannot be renamed. All documentation, examples, paths, sidebars, build commands, CI scripts, and future automation flows must follow this exact folder structure.

## Project Objective

Create Module 1 textbook content using Spec-Kit Plus and Claude Code, integrate content into docusaurus template already in repo, produce deployable docusaurus docs for module1, no RAG or backend work in this project.

## In-Scope

- Create the following markdown files under docusaurus/docs/module1/:
  - intro.md
  - ros-overview.md
  - nodes-topics-services.md
  - rclpy-basics.md
  - urdf-intro.md
  - lab1-pubsub.md
  - lab2-controller.md
  - quiz.md
  - references.md
  - appendix-hardware-lite.md
- Each generated markdown must include JSON frontmatter at top with keys: doc_id, title, module, estimated_tokens. Use doc_id pattern: project1_m1_ch01, project1_m1_ch02, ...
- Update or add a sidebar entry snippet for Module 1 in the spec, pointing to docusaurus/docs/module1/* files, so downstream automation can update docusaurus/sidebars.js.
- Add per-doc metadata placeholders for future RAG fields, keys: embedding_required: true, chunk_hint_tokens: 500

## Out-of-Scope

- No RAG backend implementation, no FastAPI, no Qdrant, no Neon, no OpenAI/Claude agent wiring for this project.
- No Better-Auth, no personalization, no Urdu translation, no subagents, no Isaac Sim heavy demos. Only conceptual notes about simulation allowed.

## Deliverables

- docusaurus/docs/module1/*.md as listed above
- spec.md updated in-place
- README.md short section: commands to build and preview docusaurus locally
- Acceptance checklist block inside spec.md listing exact pass/fail checks

## Acceptance Criteria

- Local render: running `cd docusaurus && npm run start` must show Module 1 pages, no build errors.
- Build: `npm run build` must succeed, `npm run serve` must serve static files from docusaurus/build/.
- Chapter content: every chapter must include
  - Objective (1-2 lines)
  - 2-3 Learning outcomes
  - At least one hands-on lab with step-by-step commands
  - At least one code snippet, <= 20 lines for examples
  - One or more exercises or quiz questions
- Sidebar: Module 1 appears in sidebars in correct order
- Frontmatter: each doc contains the JSON frontmatter keys required
- File size: each chapter < 3000 words
- README: contains build, preview, and deploy hint for GitHub Pages or Vercel

## Machine Friendly Instructions for Downstream Automation

### Doc Generation Map

| doc_id | Path | Content Brief |
|--------|------|---------------|
| project1_m1_ch01 | docusaurus/docs/module1/intro.md | Introduction to ROS 2 and its role in robotic nervous systems |
| project1_m1_ch02 | docusaurus/docs/module1/ros-overview.md | Overview of ROS 2 architecture and core concepts |
| project1_m1_ch03 | docusaurus/docs/module1/nodes-topics-services.md | Detailed explanation of nodes, topics, and services |
| project1_m1_ch04 | docusaurus/docs/module1/rclpy-basics.md | Basic Python client library (rclpy) usage |
| project1_m1_ch05 | docusaurus/docs/module1/urdf-intro.md | Introduction to Universal Robot Description Format |
| project1_m1_ch06 | docusaurus/docs/module1/lab1-pubsub.md | Hands-on lab for publisher-subscriber pattern |
| project1_m1_ch07 | docusaurus/docs/module1/lab2-controller.md | Hands-on lab for implementing robot controllers |
| project1_m1_ch08 | docusaurus/docs/module1/quiz.md | Quiz questions to assess understanding of Module 1 |
| project1_m1_ch09 | docusaurus/docs/module1/references.md | References and further reading materials |
| project1_m1_ch10 | docusaurus/docs/module1/appendix-hardware-lite.md | Appendix with lightweight hardware recommendations |

### Doc ID Naming Rules

- Use the pattern: project1_m1_chXX where XX is the chapter number
- Example: project1_m1_ch01 = intro.md
- Example: project1_m1_ch02 = ros-overview.md

### Embedding Placeholder Metadata

Each generated markdown file must include these metadata lines:

```yaml
embedding_required: true
chunk_hint_tokens: 500
```

## Risks and Mitigations

- Risk: Claude Code may generate technical inaccuracies, Mitigation: add human reviewer step in next plan to verify labs and code
- Risk: Docusaurus render errors for MDX, Mitigation: constrain content to plain markdown compatible with Docusaurus default

## Acceptance Checklist

- [ ] All 10 markdown files created under docusaurus/docs/module1/
- [ ] Each file includes required JSON frontmatter (doc_id, title, module, estimated_tokens)
- [ ] Each file includes embedding_required: true and chunk_hint_tokens: 500
- [ ] Each chapter meets content requirements (objectives, outcomes, labs, code snippets, exercises)
- [ ] Each chapter is under 3000 words
- [ ] Docusaurus builds successfully: `npm run build`
- [ ] Docusaurus serves locally: `cd docusaurus && npm run start`
- [ ] Module 1 appears in sidebar with correct navigation
- [ ] README includes build, preview, and deploy instructions

## Acceptance Sign-off

### Writer
- [ ] Content objectives and learning outcomes are clear
- [ ] Chapter content follows pedagogical standards
- [ ] Labs are practical and testable

### Developer
- [ ] Docusaurus integration is correct
- [ ] Frontmatter metadata is properly formatted
- [ ] Build process works as expected

### Reviewer
- [ ] Technical accuracy verified
- [ ] Content is pedagogically sound
- [ ] Acceptance criteria met

## Sidebar Snippet
```json
{
  "module1": [
    {
      "type": "category",
      "label": "Module 1: Robotic Nervous System (ROS 2)",
      "items": [
        "module1/intro",
        "module1/ros-overview",
        "module1/nodes-topics-services",
        "module1/rclpy-basics",
        "module1/urdf-intro",
        "module1/lab1-pubsub",
        "module1/lab2-controller",
        "module1/quiz",
        "module1/references",
        "module1/appendix-hardware-lite"
      ]
    }
  ]
}
```

Update: Enforced permanent Docusaurus root folder naming convention ('docusaurus').

Next step: Run qwen CLI prompt to generate plans/project-1-plan.md