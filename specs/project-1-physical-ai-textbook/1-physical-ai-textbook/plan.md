---
plan_id: project-1-plan
project_ref: project-1
version: 1.0.0
author: Ahmed Raza
date: 2025-12-12
based_on_spec: spec.md
---

# Implementation Plan: Physical AI Project 1 - Module 1: Robotic Nervous System (ROS 2)

## Global Convention
The root folder for the textbook must be named 'docusaurus' and must remain unchanged across all phases of the project.

The Docusaurus root folder name is permanently 'docusaurus'. It cannot be renamed. All documentation, examples, paths, sidebars, build commands, CI scripts, and future automation flows must follow this exact folder structure.

## 1. Project Summary

Project 1 focuses on generating Module 1 textbook content using Spec-Kit Plus and Claude Code. This involves creating docusaurus markdown files for the ROS 2 fundamentals module, updating sidebars with proper navigation, adding required JSON frontmatter to each document, running local build tests, and preparing deploy-ready documentation. This plan covers only the textbook content creation for Module 1 and explicitly excludes RAG functionality, backend services, authentication, translation, and any bonus features.

## 2. Technical Context

- **Frontend Framework**: Docusaurus v3.x for static site generation
- **Content Generation**: Claude Code and Spec-Kit Plus for AI-assisted content creation
- **Documentation Format**: Markdown with JSON frontmatter
- **Hosting**: GitHub Pages or Vercel deployment
- **Module Topic**: Robotic Nervous System (ROS 2)
- **docusaurus Setup**: Existing template in repo needs to be configured for Module 1

## 3. Constitution Check

This plan aligns with the Physical AI & Humanoid Robotics Hackathon Constitution:
- Educational Excellence: Creating pedagogically sound content for ROS 2 fundamentals
- AI-Native Integration: Using Claude Code and Spec-Kit Plus for content generation
- Practical Application Focus: Including hands-on labs and exercises in the content
- Modularity and Extensibility: Creating modular chapter content that fits into the broader curriculum
- Technical Innovation: Covering cutting-edge topics in robotics middleware (ROS 2)

## 4. Milestones

### M1: Setup and wiring of docusaurus template for Module 1
- **milestone_id**: M1
- **description**: Configure docusaurus template for Module 1, create directory structure, set up basic configuration
- **duration estimate**: 1 day
- **required inputs**: docusaurus template in repo, Module 1 requirements from spec
- **expected outputs**: docusaurus configured with module1 directory and basic structure
- **acceptance conditions**: docusaurus runs locally with placeholder Module 1 pages

### M2: Generate all Module 1 chapter drafts using Spec-Kit Plus + Claude Code
- **milestone_id**: M2
- **description**: Generate draft content for all 10 Module 1 chapters using AI tools
- **duration estimate**: 3 days
- **required inputs**: Complete Module 1 spec with chapter requirements, Claude Code access
- **required inputs**: Spec-Kit Plus configuration
- **expected outputs**: 10 markdown files with draft content meeting spec requirements
- **acceptance conditions**: Each chapter includes objectives, learning outcomes, and basic content

### M3: Add labs, exercises, code examples, and finalize content
- **milestone_id**: M3
- **description**: Enhance chapter drafts with hands-on labs, exercises, and technical examples
- **duration estimate**: 3 days
- **required inputs**: Draft chapter content from M2, lab requirements from spec
- **expected outputs**: Complete content with labs, exercises, and code examples for each chapter
- **acceptance conditions**: All chapters include required components per spec (labs, code snippets, exercises)

### M4: Local testing, docusaurus build, fix rendering issues, add sidebar entries
- **milestone_id**: M4
- **description**: Test docusaurus build, fix any rendering issues, add proper sidebar navigation
- **duration estimate**: 2 days
- **required inputs**: Complete content from M3, docusaurus configuration
- **expected outputs**: Working docusaurus site with proper navigation
- **acceptance conditions**: Site builds without errors, navigation works correctly, all chapters render properly

### M5: Acceptance testing, cleanup, finalize for deployment
- **milestone_id**: M5
- **description**: Perform final testing against acceptance criteria, cleanup any issues
- **duration estimate**: 1 day
- **required inputs**: Working site from M4, acceptance criteria from spec
- **expected outputs**: Production-ready content ready for deployment
- **acceptance conditions**: All acceptance criteria from spec are met, content is under 3000 words per chapter

## 5. Detailed Workflow Steps

1. **Create directory structure**: Create `docusaurus/docs/module1/` directory if it doesn't exist
2. **Generate markdown files**: Create the following files using Claude Code:
   - `docusaurus/docs/module1/intro.md`
   - `docusaurus/docs/module1/ros-overview.md`
   - `docusaurus/docs/module1/nodes-topics-services.md`
   - `docusaurus/docs/module1/rclpy-basics.md`
   - `docusaurus/docs/module1/urdf-intro.md`
   - `docusaurus/docs/module1/lab1-pubsub.md`
   - `docusaurus/docs/module1/lab2-controller.md`
   - `docusaurus/docs/module1/quiz.md`
   - `docusaurus/docs/module1/references.md`
   - `docusaurus/docs/module1/appendix-hardware-lite.md`

3. **Required JSON frontmatter example**:
   ```json
   ---
   doc_id: project1_m1_ch01
   title: "Introduction to ROS 2"
   module: "Module 1: Robotic Nervous System"
   estimated_tokens: 800
   embedding_required: true
   chunk_hint_tokens: 500
   ---
   ```

4. **Naming convention**: Use the pattern `project1_m1_chXX` where XX is the chapter number (01-10)

5. **Sidebar update method**: Update `docusaurus/sidebars.js` to include Module 1 entries in the correct order

6. **Docusaurus commands**:
   - `npm install` - Install dependencies
   - `cd docusaurus && npm run start` - Start local development server
   - `cd docusaurus && npm run build` - Build static site
   - `cd docusaurus && npm run serve` - Serve built site locally

7. **Word limit rules**: Each document must be under 3000 words to ensure readability and performance

8. **Lab + quiz structure**: Each lab chapter must include step-by-step instructions with command examples, and quiz chapters must include multiple-choice and practical questions

9. **Content validation rules**:
   - All code snippets must be verified for accuracy
   - Technical concepts must align with current ROS 2 documentation
   - Each chapter must include learning objectives and outcomes

10. **Final testing**: Run acceptance tests to ensure all criteria are met before deployment

## 6. Roles & Responsibilities

### M1: Setup and wiring
- **dev**: Sets up docusaurus configuration, creates directory structure

### M2: Generate chapter drafts
- **writer**: Uses Claude Code and Spec-Kit Plus to generate content drafts
- **dev**: Ensures proper frontmatter and file structure

### M3: Add labs, exercises, code examples
- **writer**: Adds labs, exercises, and code examples to each chapter
- **reviewer**: Reviews technical accuracy of code examples and labs

### M4: Local testing and build
- **dev**: Tests docusaurus build process, fixes rendering issues, updates sidebar
- **writer**: Reviews content for consistency and completeness

### M5: Acceptance testing
- **dev**: Performs final build and deployment testing
- **reviewer**: Validates content against acceptance criteria

## 7. Risk Register

### Risk 1: Claude Code generates technically inaccurate content
- **Impact**: Learning objectives not met, incorrect technical information provided
- **Mitigation**: Add human reviewer step to verify all technical content, especially labs and code examples

### Risk 2: Docusaurus render errors for complex markdown
- **Impact**: Content may not display properly, breaking user experience
- **Mitigation**: Test each chapter individually during development, stick to basic markdown features

### Risk 3: Chapter content exceeds 3000 words limit
- **Impact**: Performance issues, longer load times, poor user experience
- **Mitigation**: Regularly monitor document length during creation, break down complex topics into multiple sections if needed

## 8. Resource Requirements

- **Node.js**: Version 18.x or higher for Docusaurus compatibility
- **Docusaurus**: Version 3.x installed via npm
- **Claude Code**: Access to Claude with sufficient context window for textbook content
- **Spec-Kit Plus**: Properly configured for content generation workflows
- **Local testing**: Commands to build and serve docusaurus locally
- **Git**: For version control of content changes

## 9. QA Checklist (Acceptance Criteria derived from spec.md)

### Chapter completeness
- [ ] All 10 chapters created per specification
- [ ] Each chapter includes objectives (1-2 lines)
- [ ] Each chapter includes 2-3 learning outcomes
- [ ] Content aligns with chapter descriptions in spec

### Labs and exercises
- [ ] Lab chapters include hands-on steps with command examples
- [ ] Code snippets provided (≤20 lines each)
- [ ] Exercises and quiz questions included in relevant chapters

### Technical requirements
- [ ] Each chapter <3000 words
- [ ] docusaurus builds successfully with `cd docusaurus && npm run build`
- [ ] Local server works with `cd docusaurus && npm run start`
- [ ] No build errors or warnings

### Frontmatter and metadata
- [ ] Each file includes JSON frontmatter with required keys
- [ ] doc_id follows pattern project1_m1_chXX
- [ ] embedding_required: true included
- [ ] chunk_hint_tokens: 500 included

### Navigation
- [ ] Sidebar entries created for Module 1 in docusaurus/sidebars.js
- [ ] Navigation appears in correct order
- [ ] All links work correctly

### README
- [ ] README updated with build and deployment instructions
- [ ] Commands for local development included

Update: Enforced permanent Docusaurus root folder naming convention ('docusaurus').

## 10. Automation Notes

### Sidebar Snippet
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

### Next Step
After this plan.md is approved by stakeholders, the next step is to generate the tasks.md file using the qwen CLI. Run the following command from the project root:

`qwen generate-tasks`

This will automatically create a detailed task list based on the milestones and workflow steps defined in this plan, preparing the work for execution by the development team.

The tasks.md file will contain granular items for each milestone that can be assigned, tracked, and completed systematically.