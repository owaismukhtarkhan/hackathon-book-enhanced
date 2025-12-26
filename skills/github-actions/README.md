# GitHub Actions & CI/CD Skills

Reusable GitHub Actions workflows and CI/CD practices.

## Key Learnings
- Creating and troubleshooting GitHub Actions workflows
- Resolving deprecated action version issues
- Setting up automated build and deployment pipelines
- Environment variable and secret management

## Template: Docusaurus Deployment Workflow
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Run health check
        run: node health-check.js

      - name: Build website
        run: npm run build

  deploy:
    if: github.ref == 'refs/heads/main'
    runs-on: ubuntu-latest
    needs: test

    permissions:
      pages: write
      id-token: write

    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}

    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

      - uses: actions/configure-pages@v4

      - uses: actions/upload-pages-artifact@v4
        with:
          path: build

      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

## Common Issues & Solutions
- **Deprecated Action Versions**: Always use latest versions (v4 for most actions)
- **Environment Variables**: Don't fail builds if API keys missing during build time
- **Directory Structure**: Use .gitkeep files to ensure directories are tracked
- **Health Checks**: Structure checks to not block build process unnecessarily

## Best Practices
- Separate test and deploy jobs
- Use proper permissions for GitHub Pages
- Implement environment protection rules
- Cache dependencies to speed up builds
- Use semantic versioning for actions