# Docusaurus Development Skills

Reusable knowledge and templates for Docusaurus projects.

## Key Learnings
- Configuring Docusaurus for GitHub Pages deployment
- Custom theme development with CSS variables
- Managing documentation structure with modular content
- Handling environment-specific configurations

## Templates & Snippets

### Docusaurus Config Template
```js
// docusaurus.config.js
const config = {
  title: 'Your Project Title',
  tagline: 'Your project tagline',
  favicon: 'img/favicon.ico',

  // GitHub Pages deployment config
  url: 'https://your-username.github.io',
  baseUrl: '/your-repo-name/',
  organizationName: 'your-username',
  projectName: 'your-repo-name',

  // Add your presets and theme configuration
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/your-username/your-repo-name/tree/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themeConfig: {
    // Your theme configuration
  },
};
```

### Custom CSS Theme Template
```css
/* src/css/custom.css */
:root {
  --ifm-color-primary: #8a2be2; /* Purple */
  --ifm-color-primary-dark: #7a24c7;
  --ifm-color-primary-darker: #6b1faa;
  --ifm-color-primary-darkest: #5b1a8e;
  --ifm-color-primary-light: #9a3df2;
  --ifm-color-primary-lighter: #a95df7;
  --ifm-color-primary-lightest: #b97cfc;
  --ifm-color-accent: #39ff14; /* Neon Green */
}

[data-theme='dark'] {
  --ifm-color-primary: #a95df7;
  --ifm-color-accent: #39ff14;
}
```

## Common Issues & Solutions
- Build failures due to environment variables
- Theme customization approaches
- GitHub Pages deployment configurations

## Best Practices
- Keep documentation modular
- Use consistent sidebar structures
- Implement proper dark mode support
- Optimize for mobile responsiveness