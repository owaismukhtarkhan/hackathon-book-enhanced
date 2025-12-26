<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles:
- Principle 1: [PRINCIPLE_1_NAME] → Physical AI & Spec-Driven Development
- Principle 2: [PRINCIPLE_2_NAME] → AI Governance & Tooling Standards
- Principle 3: [PRINCIPLE_3_NAME] → Beginner-First Content Creation
- Principle 4: [PRINCIPLE_4_NAME] → Production-Grade Implementation Standards
- Principle 5: [PRINCIPLE_5_NAME] → UI/UX & Accessibility Requirements
- Principle 6: [PRINCIPLE_6_NAME] → Technology Stack & Deployment Standards

Added sections:
- Learning Progression Model
- Tooling & Deployment Requirements
- Governance & Enforcement

Removed sections: None

Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated - Constitution Check section aligned
- .specify/templates/spec-template.md ⚠ pending - May need alignment with new principles
- .specify/templates/tasks-template.md ⚠ pending - May need alignment with new principles

Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### Physical AI & Spec-Driven Development
Every chapter starts as a standalone specification; All content must follow Spec-Kit Plus methodology with clear purpose, inputs, outputs, constraints, and validation criteria. All chapters must be spec-defined and validated to ensure production-grade implementation over theory.

### AI Governance & Tooling Standards
AI tools are permitted only as content refiners and editors; AI tools must not act as autonomous authors or decision-makers. Google Gemini must be used if external AI APIs are required; OpenAI APIs are explicitly disallowed. All AI-assisted content must be reviewed, verified, and explicitly aligned with its governing spec.

### Beginner-First Content Creation
All content must assume minimal prior knowledge, follow step-by-step instructional flow, avoid unexplained jargon, and prefer real, working implementations over conceptual discussion. Content must be designed for absolute beginners with production-grade examples.

### Production-Grade Implementation Standards
All code and implementations must be production-grade, follow industry best practices, be buildable and deployable, use clear naming and structure. Code that does not run or cannot be validated is non-compliant. JavaScript is the default language; TypeScript may be used only if explicitly required and justified in the governing spec.

### UI/UX & Accessibility Requirements
A Purple + Neon theme must be applied consistently across pages, components, navigation, and UI elements. All images and screenshots must include meaningful alt text. Mobile-first design is mandatory with layouts tested across viewports from 320px to 2560px. Lazy loading is mandatory for images and media assets.

### Technology Stack & Deployment Standards
Docusaurus is mandatory for documentation generation and deployment. Markdown/MDX must follow Docusaurus standards. The book must be deployed on GitHub Pages and be publicly accessible with successful builds without errors. Mandatory tools include ROS 2, Gazebo, Unity, NVIDIA Isaac Sim & Isaac ROS, and Vision-Language-Action pipelines.

## Learning Progression Model

The book follows a progressive complexity structure with Weeks 1-2 for fundamentals, Weeks 3-10 for core skills, and Weeks 11-13 for advanced integration. Introducing advanced topics before prerequisite stages is prohibited.

## Tooling & Deployment Requirements

Docusaurus is mandatory for the book generation. Markdown/MDX must follow Docusaurus standards. The book must be deployed on GitHub Pages, be publicly accessible, and build successfully without errors.

## Governance & Enforcement

This project is a hackathon submission. Any contribution that violates this constitution must be rejected and must be removed or corrected. No exceptions are permitted for time pressure, AI-generated content, or contributor experience level. This project is a hackathon submission. Any content that violates this constitution is non-compliant and invalid.

## Governance

This constitution is the single source of truth. Any artifact that violates this constitution is considered non-compliant and invalid. All contributions must verify compliance. This constitution may only be modified through explicit version updates, clear written justification, and repository-wide compliance after amendment.

**Version**: 1.1.0 | **Ratified**: 2025-06-13 | **Last Amended**: 2025-12-16