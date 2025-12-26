# /sp.constitution.md

## 📘 AI / Spec-Driven Book Creation Constitution

**Version:** 1.1
**Status:** Binding & Enforceable
**Applies To:** All content, code, specifications, documentation, tooling, and deployments in this repository

---

## 1. Purpose

This constitution governs the creation of an **AI-assisted, Spec-Driven textbook** built using **Docusaurus** and deployed via **GitHub Pages**, produced as an **official hackathon deliverable**.

The book is explicitly designed for **absolute beginners** and prioritizes **production-grade implementation over theory**, while enforcing strict standards for specifications, validation, accessibility, performance, and deployment.

This document is the **single source of truth**. Any artifact that violates this constitution is considered **non-compliant and invalid**.

---

## 2. Target Audience

The primary audience is:

* **Absolute beginners**

All content MUST:

* Assume minimal prior knowledge
* Follow step-by-step instructional flow
* Avoid unexplained jargon
* Prefer real, working implementations over conceptual discussion

---

## 3. Core Methodology: Spec-Driven Development

### 3.1 Mandatory Use of Spec-Kit Plus

**Spec-Kit Plus is mandatory** for:

* Every chapter
* Every feature
* Every example
* Every implementation

🚫 **No chapter may exist without a corresponding spec**

Each spec MUST explicitly define:

* Purpose
* Inputs
* Outputs
* Constraints
* Validation criteria

---

## 4. AI Usage & Governance

### 4.1 Role of AI

AI tools are permitted **only** as:

* Content refiners
* Editors
* Structure improvers

AI tools MUST NOT act as autonomous authors or decision-makers.

### 4.2 Approved AI & APIs

* If AI APIs are required, **Google Gemini MUST be used**
* **OpenAI APIs are explicitly disallowed**

Allowed usage includes:

* Refining explanations
* Improving clarity
* Formatting content

🚫 **No AI output without human validation**

All AI-assisted content MUST be:

* Reviewed
* Verified
* Explicitly aligned with its governing spec

---

## 5. Writing & Content Standards

### 5.1 Writing Style

* Minimal theory
* Maximum implementation
* Step-by-step execution
* Beginner-first explanations
* Production-grade examples

### 5.2 Prohibited Content

The following are strictly forbidden:

* Long theoretical essays
* Placeholder or dummy code
* Pseudocode without execution context
* Unverified or hallucinated AI output

---

## 6. Code & Technical Quality Standards

All code and implementations MUST:

* Be production-grade
* Follow industry best practices
* Be buildable and deployable
* Use clear naming and structure

Code that does not run or cannot be validated is non-compliant.

## 6.1 Language Usage Rule (JavaScript / TypeScript)

*JavaScript is the default language
*TypeScript MAY be used only if explicitly required
*The requirement for TypeScript MUST be:

  * Justified in the governing spec
  * Clearly stated before implementation
  * Necessary for correctness, safety, or tooling compatibility

🚫 **TypeScript MUST NOT be used by default or for preference alone**

---

## 7. UI / UX, Accessibility & Performance Rules

### 7.1 Visual Identity

* A **Purple + Neon** theme MUST be applied consistently across:

  * Pages
  * Components
  * Navigation
  * UI elements

### 7.2 Accessibility

* All images and screenshots MUST include meaningful **alt text**
* Interactive elements MUST follow accessibility best practices

🚫 **No screenshots without alt text**

### 7.3 Performance

* **Lazy loading is mandatory** for:

  * Images
  * Media assets

### 7.4 Responsiveness

* Mobile-first design is mandatory
* Layouts MUST be tested across viewports:

  * **320px → 2560px**

---

## 8. Learning Progression Model

The book MUST follow a progressive complexity structure:

* **Weeks 1–2:** Fundamentals
* **Weeks 3–10:** Core skills
* **Weeks 11–13:** Advanced integration

🚫 Introducing advanced topics before prerequisite stages is prohibited.

---

## 9. Tooling & Deployment Requirements

### 9.1 Tooling

* **Docusaurus** is mandatory
* Markdown / MDX MUST follow Docusaurus standards

### 9.2 Deployment

The book MUST:

* Be deployed on **GitHub Pages**
* Be publicly accessible
* Build successfully without errors

---

## 10. Governance & Enforcement

* This project is a **hackathon submission**
* Any contribution that violates this constitution:

  * MUST be rejected
  * MUST be removed or corrected

No exceptions are permitted for:

* Time pressure
* AI-generated content
* Contributor experience level

---

## 11. Success Criteria

The project is considered successful ONLY if:

* ✅ Fully deployed using Docusaurus
* ✅ Publicly accessible via GitHub Pages
* ✅ All chapters are spec-driven
* ✅ All AI output is validated
* ✅ All constitutional rules are enforced

---

## 12. Amendment Policy

This constitution MAY ONLY be modified through:

* Explicit version updates
* Clear written justification
* Repository-wide compliance after amendment

---

**By contributing to this repository, you explicitly agree to follow this constitution.**
