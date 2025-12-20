---
plan_id: module-2-plan
project_ref: module-2
version: 1.0.0
author: Ahmed Raza
date: 2025-12-13
based_on_spec: spec.md
---

# Implementation Plan: Physical AI Module 2 - The Digital Twin (Gazebo & Unity)

## Global Convention
The root folder for the textbook must be named 'docusaurus' and must remain unchanged across all phases of the project.

The Docusaurus root folder name is permanently 'docusaurus'. It cannot be renamed. All documentation, examples, paths, sidebars, build commands, CI scripts, and future automation flows must follow this exact folder structure.

## 1. Project Summary

Module 2 focuses on generating textbook content about Digital Twins using Gazebo and Unity for the Physical AI & Humanoid Robotics curriculum. This involves creating docusaurus markdown files covering simulation fundamentals, physics simulation, URDF/SDF usage, sensor simulation, Unity integration, human-robot interaction simulation, and sim-to-real understanding. This plan covers only the textbook content creation for Module 2 and explicitly excludes RAG functionality, backend services, authentication, translation, and any bonus features.

## 2. Technical Context

- **Frontend Framework**: Docusaurus v3.x for static site generation
- **Content Generation**: Claude Code and Spec-Kit Plus for AI-assisted content creation
- **Documentation Format**: Markdown with JSON frontmatter
- **Hosting**: GitHub Pages or Vercel deployment
- **Module Topic**: Digital Twins (Gazebo & Unity)
- **docusaurus Setup**: Existing template already configured from Module 1
- **Simulation Context**: Content builds upon ROS 2 concepts from Module 1

## 3. Constitution Check

This plan aligns with the Physical AI & Humanoid Robotics Hackathon Constitution:
- Educational Excellence: Creating pedagogically sound content for digital twin simulation
- AI-Native Integration: Using Claude Code and Spec-Kit Plus for content generation
- Practical Application Focus: Including hands-on labs and exercises in the content
- Modularity and Extensibility: Creating modular chapter content that fits into the broader curriculum
- Technical Innovation: Covering cutting-edge topics in robotics simulation (Gazebo, Unity)

## 4. Milestones

### M1: Setup and wiring of docusaurus template for Module 2
- **milestone_id**: M1
- **description**: Configure docusaurus template for Module 2, create directory structure, set up basic configuration
- **duration estimate**: 0.5 day
- **required inputs**: Module 2 spec with chapter requirements, existing docusaurus setup from Module 1
- **expected outputs**: docusaurus configured with module2 directory and basic structure
- **acceptance conditions**: docusaurus runs locally with placeholder Module 2 pages

### M2: Generate all Module 2 chapter drafts using Spec-Kit Plus + Claude Code
- **milestone_id**: M2
- **description**: Generate draft content for all 7 Module 2 chapters using AI tools
- **duration estimate**: 2 days
- **required inputs**: Complete Module 2 spec with chapter requirements, Claude Code access
- **required inputs**: Spec-Kit Plus configuration
- **expected outputs**: 7 markdown files with draft content meeting spec requirements
- **acceptance conditions**: Each chapter includes objectives, learning outcomes, and basic content

### M3: Add labs, exercises, simulation examples, and finalize content
- **milestone_id**: M3
- **description**: Enhance chapter drafts with hands-on labs, exercises, and simulation examples
- **duration estimate**: 3 days
- **required inputs**: Draft chapter content from M2, lab requirements from spec
- **expected outputs**: Complete content with labs, exercises, and simulation examples for each chapter
- **acceptance conditions**: All chapters include required components per spec (labs, simulation exercises, learning outcomes)

### M4: Local testing, docusaurus build, fix rendering issues, add sidebar entries
- **milestone_id**: M4
- **description**: Test docusaurus build, fix any rendering issues, add proper sidebar navigation
- **duration estimate**: 1 day
- **required inputs**: Complete content from M3, docusaurus configuration
- **expected outputs**: Working docusaurus site with proper navigation
- **acceptance conditions**: Site builds without errors, navigation works correctly, all chapters render properly

### M5: Acceptance testing, cleanup, finalize for integration with Module 1
- **milestone_id**: M5
- **description**: Perform final testing against acceptance criteria, cleanup any issues
- **duration estimate**: 0.5 day
- **required inputs**: Working site from M4, acceptance criteria from spec
- **expected outputs**: Production-ready content ready for deployment
- **acceptance conditions**: All acceptance criteria from spec are met, content is under 3000 words per chapter

## 5. Detailed Workflow Steps

1. **Create directory structure**: Create `docusaurus/docs/module2/` directory if it doesn't exist
2. **Generate markdown files**: Create the following files using Claude Code:
   - `docusaurus/docs/module2/intro.md`
   - `docusaurus/docs/module2/gazebo-fundamentals.md`
   - `docusaurus/docs/module2/physics-simulation.md`
   - `docusaurus/docs/module2/urdf-sdf-usage.md`
   - `docusaurus/docs/module2/sensor-simulation.md`
   - `docusaurus/docs/module2/unity-integration.md`
   - `docusaurus/docs/module2/hri-simulation.md`
   - `docusaurus/docs/module2/sim-to-real.md`

3. **Required JSON frontmatter example**:
   ```json
   ---
   doc_id: module2_m2_ch01
   title: "Introduction to Digital Twins"
   module: "Module 2: The Digital Twin (Gazebo & Unity)"
   estimated_tokens: 800
   embedding_required: true
   chunk_hint_tokens: 500
   ---
   ```

4. **Naming convention**: Use the pattern `module2_m2_chXX` where XX is the chapter number (01-07)

5. **Sidebar update method**: Update `docusaurus/sidebars.js` to include Module 2 entries in the correct order

6. **Docusaurus commands**:
   - `cd docusaurus && npm run start` - Start local development server
   - `cd docusaurus && npm run build` - Build static site
   - `cd docusaurus && npm run serve` - Serve built site locally

7. **Word limit rules**: Each document must be under 3000 words to ensure readability and performance

8. **Lab + exercise structure**: Each lab chapter must include step-by-step simulation instructions with environment setup examples, and exercise chapters must include practical simulation tasks

9. **Content validation rules**:
   - All simulation examples must align with current Gazebo/Unity documentation
   - Technical concepts must connect properly with ROS 2 concepts from Module 1
   - Each chapter must include learning objectives and outcomes

10. **Final testing**: Run acceptance tests to ensure all criteria are met before deployment

## 6. Roles & Responsibilities

### M1: Setup and wiring
- **dev**: Sets up docusaurus configuration, creates directory structure

### M2: Generate chapter drafts
- **writer**: Uses Claude Code and Spec-Kit Plus to generate content drafts
- **dev**: Ensures proper frontmatter and file structure

### M3: Add labs, exercises, simulation examples
- **writer**: Adds labs, exercises, and simulation examples to each chapter
- **reviewer**: Reviews technical accuracy of simulation examples and labs

### M4: Local testing and build
- **dev**: Tests docusaurus build process, fixes rendering issues, updates sidebar
- **writer**: Reviews content for consistency and completeness

### M5: Acceptance testing
- **dev**: Performs final build and deployment testing
- **reviewer**: Validates content against acceptance criteria

## 7. Risk Register

### Risk 1: Claude Code generates technically inaccurate simulation content
- **Impact**: Learning objectives not met, incorrect simulation techniques provided
- **Mitigation**: Add human reviewer step with simulation expertise to verify all technical content, especially simulation examples and labs

### Risk 2: Docusaurus render errors for complex markdown with simulation diagrams
- **Impact**: Content may not display properly, breaking user experience
- **Mitigation**: Test each chapter individually during development, stick to basic markdown features, use standard image inclusion methods for simulation screenshots

### Risk 3: Chapter content exceeds 3000 words limit
- **Impact**: Performance issues, longer load times, poor user experience
- **Mitigation**: Regularly monitor document length during creation, break down complex simulation topics into multiple sections if needed

## 8. Resource Requirements

- **Node.js**: Version 18.x or higher for Docusaurus compatibility
- **Docusaurus**: Version 3.x installed via npm (from Module 1)
- **Claude Code**: Access to Claude with sufficient context window for textbook content
- **Spec-Kit Plus**: Properly configured for content generation workflows
- **Simulation Software**: Gazebo, Unity access for content verification
- **ROS 2**: For context and integration examples
- **Local testing**: Commands to build and serve docusaurus locally
- **Git**: For version control of content changes

## 9. QA Checklist (Acceptance Criteria derived from spec.md)

### Chapter completeness
- [ ] All 7 chapters created per specification
- [ ] Each chapter includes objectives (1-2 lines)
- [ ] Each chapter includes 2-3 learning outcomes
- [ ] Content aligns with chapter descriptions in spec

### Labs and exercises
- [ ] Lab chapters include hands-on simulation steps with environment setup
- [ ] Simulation examples provided with explanations
- [ ] Exercises and practical tasks included in relevant chapters

### Technical requirements
- [ ] Each chapter <3000 words
- [ ] docusaurus builds successfully with `cd docusaurus && npm run build`
- [ ] Local server works with `cd docusaurus && npm run start`
- [ ] No build errors or warnings

### Frontmatter and metadata
- [ ] Each file includes JSON frontmatter with required keys
- [ ] doc_id follows pattern module2_m2_chXX
- [ ] embedding_required: true included
- [ ] chunk_hint_tokens: 500 included

### Navigation
- [ ] Sidebar entries created for Module 2 in docusaurus/sidebars.js
- [ ] Navigation appears in correct order
- [ ] All links work correctly

### Content accuracy
- [ ] Simulation examples align with current Gazebo documentation
- [ ] Unity integration examples are accurate
- [ ] Physics simulation concepts are correctly explained
- [ ] URDF/SDF usage examples are valid

Update: Enforced permanent Docusaurus root folder naming convention ('docusaurus').

## 10. Automation Notes

### Sidebar Snippet
```json
{
  "module2": [
    {
      "type": "category",
      "label": "Module 2: The Digital Twin (Gazebo & Unity)",
      "items": [
        "module2/intro",
        "module2/gazebo-fundamentals",
        "module2/physics-simulation",
        "module2/urdf-sdf-usage",
        "module2/sensor-simulation",
        "module2/unity-integration",
        "module2/hri-simulation",
        "module2/sim-to-real"
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