# Version Control Skills

Best practices for Git and repository management.

## Key Learnings
- Branch management and comparison
- Resolving merge conflicts and tracking changes
- Commit message best practices
- Repository organization

## Git Workflows

### Feature Branch Workflow
```bash
# Create feature branch
git checkout -b feature/new-feature

# Make changes and commit
git add .
git commit -m "feat: add new feature with description"

# Push and create pull request
git push origin feature/new-feature
```

### Commit Message Best Practices
```
feat: add new authentication system
^--^  ^------------------------^
|     |
|     +-> Summary in present tense
|
+-------> Type: feat, fix, docs, style, refactor, test, chore
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only changes
- `style`: Formatting, missing semi-colons, etc.
- `refactor`: Code change that neither fixes a bug nor adds a feature
- `test`: Adding missing tests
- `chore`: Other changes that don't modify src or test files

## Repository Organization

### Using .gitkeep for Directory Tracking
```bash
# Create empty directories that Git normally ignores
mkdir -p src/services
touch src/services/.gitkeep
git add src/services/.gitkeep
```

### Common Git Commands
```bash
# Check status
git status

# Compare branches
git log --oneline --graph --all

# View differences
git diff HEAD~1

# Undo changes
git checkout -- file.js

# Reset to previous commit
git reset --hard HEAD~1
```

## Best Practices
- Use descriptive commit messages
- Keep commits focused on single changes
- Use .gitkeep files for empty directories
- Regularly sync with remote repository
- Use feature branches for development
- Clean up old branches after merging