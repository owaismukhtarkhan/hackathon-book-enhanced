# Quality Assurance Skills

Testing, validation, and quality assurance practices.

## Key Learnings
- Health check system implementation
- Automated testing and validation
- Success threshold management for educational modules
- Comprehensive validation frameworks

## Testing Patterns

### Health Check System
```javascript
class HealthChecker {
  constructor() {
    this.results = [];
    this.passed = 0;
    this.failed = 0;
  }

  async runAllChecks() {
    const checks = [
      this.checkDependencies(),
      this.checkConfiguration(),
      this.checkEnvironment(),
      this.checkBuildReadiness()
    ];

    const results = await Promise.all(checks);
    return this.evaluateResults(results);
  }

  async checkDependencies() {
    const name = 'Dependencies';
    let passed = true;
    const errors = [];

    if (!fs.existsSync(path.join(process.cwd(), 'package.json'))) {
      errors.push('package.json not found');
      passed = false;
    }

    this.addResult(name, passed, errors);
  }

  addResult(name, passed, errors = []) {
    this.results.push({ name, passed, errors });
    if (passed) this.passed++;
    else this.failed++;
  }

  evaluateResults(results) {
    const total = results.length;
    const passed = results.filter(r => r.passed).length;
    return {
      success: passed === total,
      summary: { total, passed, failed: total - passed }
    };
  }
}
```

### Validation Framework
```javascript
class ValidationFramework {
  validateStructure(expectedStructure) {
    const errors = [];

    for (const [path, type] of Object.entries(expectedStructure)) {
      const fullPath = path.join(process.cwd(), path);
      if (!fs.existsSync(fullPath)) {
        errors.push(`Missing: ${path}`);
      } else if (type === 'file' && fs.statSync(fullPath).isDirectory()) {
        errors.push(`Expected file but found directory: ${path}`);
      } else if (type === 'directory' && !fs.statSync(fullPath).isDirectory()) {
        errors.push(`Expected directory but found file: ${path}`);
      }
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  validateContent(content, requirements) {
    const missing = [];

    for (const req of requirements) {
      if (!content.includes(req)) {
        missing.push(req);
      }
    }

    return {
      meetsRequirements: missing.length === 0,
      missing,
      completeness: (requirements.length - missing.length) / requirements.length
    };
  }
}
```

## Assessment and Success Metrics

### Educational Assessment Template
```javascript
class AssessmentValidator {
  constructor(config) {
    this.config = {
      successThreshold: config.successThreshold || 0.7, // 70%
      ...config
    };
  }

  validateStudentSubmission(submission, requirements) {
    const metrics = this.evaluateMetrics(submission, requirements);
    const score = this.calculateScore(metrics);

    return {
      score,
      passed: score >= this.config.successThreshold,
      feedback: this.generateFeedback(metrics, requirements),
      metrics
    };
  }

  calculateScore(metrics) {
    // Weight different metrics appropriately
    return (
      metrics.completion * 0.4 +
      metrics.correctness * 0.4 +
      metrics.style * 0.2
    );
  }
}
```

## Quality Assurance Best Practices

### Automated Checks
- Directory structure validation
- File content validation
- Dependency verification
- Configuration validation
- Build readiness checks

### Success Thresholds
- Define clear success criteria
- Implement measurable metrics
- Provide actionable feedback
- Track progress over time
- Set appropriate thresholds for different contexts

### Testing Strategies
- Unit tests for individual components
- Integration tests for system behavior
- Health checks for system readiness
- Performance benchmarks
- Accessibility validation

## Best Practices
- Implement comprehensive health checks
- Set clear success thresholds
- Provide meaningful feedback
- Test in multiple environments
- Automate validation where possible
- Monitor system health continuously
- Document validation criteria clearly