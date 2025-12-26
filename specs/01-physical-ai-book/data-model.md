# Data Model: Physical AI & Humanoid Robotics Book

## Entities

### Module Content
- **name**: String - The name of the module (e.g., "ROS 2 Fundamentals")
- **description**: String - Brief description of the module content
- **weekRange**: String - The weeks this module covers (e.g., "Weeks 3-5")
- **prerequisites**: [String] - List of prerequisite modules that must be completed first
- **learningObjectives**: [String] - List of learning objectives for the module
- **contentSections**: [ContentSection] - List of sections within the module
- **codeExamples**: [CodeExample] - List of code examples provided in the module
- **assessments**: [Assessment] - List of assessments for the module

### ContentSection
- **title**: String - Title of the content section
- **content**: String - The main content in Markdown format
- **learningOutcomes**: [String] - List of specific learning outcomes
- **duration**: Number - Estimated time to complete in minutes
- **dependencies**: [String] - List of dependencies on other sections

### CodeExample
- **title**: String - Title of the code example
- **language**: String - Programming language (TypeScript, Python, etc.)
- **code**: String - The actual code content
- **explanation**: String - Explanation of the code functionality
- **expectedOutput**: String - What the code should produce
- **moduleRef**: String - Reference to the parent module

### Assessment
- **title**: String - Title of the assessment
- **type**: String - Type of assessment (e.g., "quiz", "coding", "project")
- **requirements**: [String] - List of requirements to pass the assessment
- **successThreshold**: Number - Minimum score required to pass (from spec: 90%, 85%, 80%, 95%)
- **validationMethod**: String - How the assessment will be validated (automated code review and execution tests)

### HardwareSpecification
- **name**: String - Name of the hardware component (e.g., "Digital Twin Workstation")
- **category**: String - Category (e.g., "Workstation", "Edge AI Kit", "Robot Lab")
- **minimumRequirements**: [String] - List of minimum requirements
- **recommendedRequirements**: [String] - List of recommended requirements
- **compatibilityNotes**: String - Any compatibility considerations

### CapstoneProject
- **title**: String - Title of the capstone project ("The Autonomous Humanoid")
- **requirements**: [String] - All 6 specific requirements that must be fulfilled
- **description**: String - Detailed description of the project
- **prerequisites**: [String] - Modules that must be completed before starting
- **successCriteria**: String - Criteria for successful completion (all 6 requirements fulfilled)

## Relationships

- Module Content contains multiple Content Sections and Code Examples
- Module Content has multiple Assessments
- Content Sections may depend on other Content Sections
- Code Examples are associated with specific Modules
- Assessments are linked to specific Modules
- Capstone Project has 6 specific requirements that must all be implemented

## Validation Rules

- Module Content name must be unique within the curriculum
- Week ranges must follow the progressive sequence (Weeks 1-2 → Weeks 3-10 → Weeks 11-13)
- Prerequisites must be completed before advancing to dependent modules
- Assessment success thresholds must meet minimum standards (90%, 85%, 80%, 95%)
- All 6 capstone project requirements must be fulfilled for passing
- Code examples must be in TypeScript as required by constitution
- All content must be accessible to beginners (no assumed prior knowledge)