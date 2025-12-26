# Research: Physical AI & Humanoid Robotics Book

## Decision: Technology Stack Selection
**Rationale**: Based on constitution requirements, TypeScript is mandatory for all implementations and Docusaurus is required for documentation generation and deployment. The book must be deployed on GitHub Pages with a Purple + Neon theme.

**Alternatives considered**:
- JavaScript vs TypeScript: TypeScript chosen as required by constitution
- Hugo vs Docusaurus vs GitBook: Docusaurus chosen as required by constitution
- Netlify/Vercel vs GitHub Pages: GitHub Pages chosen as required by constitution

## Decision: AI API Selection
**Rationale**: Constitution mandates Google Gemini must be used if external AI APIs are required; OpenAI APIs are explicitly disallowed. Assessment validation occurs through automated code review and execution tests.

**Alternatives considered**:
- OpenAI APIs: Prohibited by constitution
- Anthropic Claude: Not specified as mandatory, Google Gemini required by constitution
- Self-hosted models: Possible but assessment validation requires Google Gemini

## Decision: Hardware Requirements
**Rationale**: Students must have access to specified hardware configurations with RTX GPU minimum requirements as per functional requirements. This ensures consistent learning experience.

**Alternatives considered**:
- Cloud-based alternatives: Not acceptable per clarification
- Minimum hardware specifications: RTX 4070 Ti (12GB VRAM) or higher as specified in original requirements
- Jetson kits for edge AI: Required as specified in original requirements

## Decision: Content Structure
**Rationale**: Follow progressive complexity structure with Weeks 1-2 fundamentals, Weeks 3-10 core skills, Weeks 11-13 advanced integration. Students must complete foundational modules before advancing to subsequent modules.

**Alternatives considered**:
- Self-paced learning without prerequisites: Not allowed per clarification
- Compressed timeline: 13-week structure mandated by original specification
- Modular approach allowing topic selection: Not allowed per clarification

## Decision: Assessment Strategy
**Rationale**: All 6 specific capstone requirements must be implemented for passing. Success criteria represent minimum acceptable standards that students must meet to pass (not aspirational targets).

**Alternatives considered**:
- Partial completion with threshold: Not allowed per clarification
- Instructor manual review: Automated validation required per clarification
- Peer review process: Automated validation required per clarification