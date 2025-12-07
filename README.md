# Physical AI & Humanoid Robotics Curriculum

This repository contains the complete curriculum for Physical AI & Humanoid Robotics, covering everything from ROS 2 fundamentals to advanced Vision-Language-Action (VLA) integration for humanoid robots.

## Curriculum Overview

The curriculum is divided into four interconnected modules:

1. **ROS 2 Fundamentals**: Core concepts of ROS 2 for humanoid robot control
2. **Simulation Environments**: Physics simulation with Gazebo and Unity digital twins
3. **AI Perception & Navigation**: Advanced perception using NVIDIA Isaac and Nav2
4. **Vision-Language-Action Integration**: Voice-to-action pipeline with cognitive planning

## Documentation Website

This curriculum includes a Docusaurus-powered documentation website that can be run locally for offline access to all curriculum materials.

### Prerequisites

- Node.js (version 18 or higher)
- npm package manager

### Installation and Setup

1. **Install dependencies:**
   ```bash
   npm install
   ```

2. **Start the development server:**
   ```bash
   npm start
   ```

   This command starts a local development server and opens the documentation in your browser at `http://localhost:3000`. Most changes are reflected live without restarting the server.

3. **Build for production:**
   ```bash
   npm run build
   ```

   This command generates static content in the `build` directory and can be served using any static hosting service.

### Directory Structure

- `docs/` - All curriculum content organized by modules
- `src/` - Source code including CSS customizations
- `static/` - Static assets like images
- `docusaurus.config.js` - Website configuration
- `sidebars.js` - Navigation sidebar definitions

### Curriculum Content

The documentation includes:

- **Learning objectives** for each module
- **Detailed explanations** of concepts and implementation
- **Code examples** for all key components
- **Exercises** with solutions
- **Assessment rubrics** for evaluation
- **Best practices** and troubleshooting guides

### Navigation

The sidebar organizes content by curriculum modules:
- Getting Started (Introduction, Setup, Quickstart)
- Module 1: ROS 2 Fundamentals
- Module 2: Simulation Environments
- Module 3: AI Perception & Navigation
- Module 4: Vision-Language-Action Integration
- Capstone Project Guidelines
- Assessment & Resources

## Development

To contribute to the curriculum documentation:

1. Make changes to the markdown files in the `docs/` directory
2. Preview changes using `npm start`
3. Build and test with `npm run build`
4. Submit changes via pull request

### Adding New Content

When adding new curriculum content:

1. Create markdown files in the appropriate module directory
2. Update `sidebars.js` to include new pages in the navigation
3. Follow the existing format for consistency
4. Include code examples where appropriate

## Technologies Used

- **Docusaurus v3**: Static site generator optimized for documentation
- **MDX**: JSX in Markdown files for advanced customization
- **React**: Component-based UI development
- **Infima**: Styling framework designed for content-centric sites

## Deployment

The documentation can be deployed to various platforms:
- GitHub Pages
- Netlify
- Vercel
- Any static hosting service

For GitHub Pages deployment, use `npm run deploy`.

## Support

For issues with the curriculum content, please create an issue in the repository. For technical issues with the documentation site, refer to the [Docusaurus documentation](https://docusaurus.io/).

## License

This curriculum is provided as educational content for learning Physical AI and humanoid robotics.