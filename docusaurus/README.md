# Physical AI & Humanoid Robotics Textbook

This is the official textbook website for the Physical AI & Humanoid Robotics course, built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Project 1 — Module 1: Robotic Nervous System (ROS 2)

This project focuses on generating Module 1 textbook content using Spec-Kit Plus and Claude Code, integrating content into Docusaurus, and producing deployable documentation for the ROS 2 fundamentals module.

## Opening the Book

To open the book locally:

```bash
npm start
```

Then visit [http://localhost:3000](http://localhost:3000) to access the book landing page and navigate to the content.

The book content can be accessed directly at `/docs/module1/intro`.

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
