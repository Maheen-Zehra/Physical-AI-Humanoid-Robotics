<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.0.0 (initial creation)
- Added sections: Core Principles (7), Additional Constraints, Development Workflow, Governance
- Templates requiring updates: N/A (initial creation)
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### Technical Accuracy
All robotics, AI, and engineering concepts must be validated from primary sources and peer-reviewed research. Every factual claim requires verification through credible academic sources, with priority given to IEEE, ACM, Nature Robotics, and arXiv publications with reputable authors.

### Clarity for Mixed Audience
Content must be accessible to computer science students, robotics beginners, and intermediate engineering learners. Writing clarity must maintain a Flesch-Kincaid grade level of 10-12 for accessibility, with step-by-step explanations that are beginner-friendly while maintaining technical depth.

### Content Reproducibility
All algorithms, examples, workflows, and robotics procedures must be replicable. Code examples must be provided in Python, ROS2, and basic robotics pseudo-code where relevant, with clear instructions for implementation and verification.

### Scientific Rigor
Prefer peer-reviewed sources, robotics research papers, and IEEE publications. All engineering diagrams and models must follow standard terminology and established conventions in the robotics field, ensuring consistency with industry standards.

### Pedagogical Quality
All chapters must include learning objectives, key concepts, summary, and review questions. Explanations must be structured as step-by-step processes that build understanding progressively, with clear connections between concepts.

### Zero Plagiarism Tolerance
All content must be original with proper APA or IEEE citations for any referenced material. Minimum 50% of sources must be peer-reviewed (IEEE, ACM, Nature Robotics, arXiv with reputable authors), with strict adherence to academic integrity standards.

### Docusaurus & Deployment Standards
The book must be formatted using Docusaurus v3 and deployed to GitHub Pages with a successful CI build. The book must build successfully using `npm run build` with no errors, and all content must be structured according to Docusaurus v3 requirements for proper rendering and navigation.

## Additional Constraints

The book must be formatted using Docusaurus v3 and deployed to GitHub Pages with a successful CI build. Minimum content requirements include 10-15 chapters, 1,200-2,000 words per chapter, for a total of 15,000-25,000 words. All diagrams must be generated or properly cited, with images stored in /static/img. The book must build successfully using `npm run build`.

Citation style must be consistent (APA or IEEE) throughout the project. All definitions must be technically correct and consistent with robotics standards. Media files must follow proper licensing requirements and be stored in designated directories.

## Development Workflow

Weekly review cycles must be conducted using Claude Code workspace. Each module/chapter must pass technical validation, clarity check, build check, and citation integrity check before being considered complete. The development process must follow spec-driven development principles with clear acceptance criteria for each chapter.

Success criteria include: all factual claims verified with APA/IEEE citations, zero plagiarism (automated + manual check), book compiles without errors on Docusaurus, GitHub Pages deployment successful, content passes technical accuracy review for AI concepts, robotics mechanisms, and control systems, all chapters follow consistent structure and quality, and at least 15 credible academic sources included.

## Governance

This constitution governs all aspects of the Physical AI & Humanoid Robotics book development. All contributions must comply with these principles. Amendments require documentation of the change, approval from project maintainers, and a migration plan for existing content. All PRs and reviews must verify compliance with technical accuracy, citation standards, and pedagogical requirements.

The constitution supersedes all other practices and serves as the authoritative source for project standards. All team members must familiarize themselves with these principles before contributing to the project.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
