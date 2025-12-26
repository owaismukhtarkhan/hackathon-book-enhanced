#!/usr/bin/env node
/**
 * Health Check System for Physical AI & Humanoid Robotics Book
 * Validates all system components and dependencies
 */

const fs = require('fs');
const path = require('path');
const https = require('https');

class HealthChecker {
  constructor() {
    this.checks = [];
    this.results = [];
    this.passed = 0;
    this.failed = 0;
  }

  async runAllChecks() {
    console.log('🏥 Running Health Checks for Physical AI & Humanoid Robotics Book...\n');

    // Check 1: Directory Structure
    await this.checkDirectoryStructure();

    // Check 2: Configuration Files
    await this.checkConfigFiles();

    // Check 3: Documentation Modules
    await this.checkDocumentationModules();

    // Check 4: Code Examples
    await this.checkCodeExamples();

    // Check 5: Service Files
    await this.checkServiceFiles();

    // Check 6: Assets
    await this.checkAssets();

    // Check 7: Environment Variables
    await this.checkEnvironmentVariables();

    // Check 8: Dependencies
    await this.checkDependencies();

    // Check 9: Docusaurus Configuration
    await this.checkDocusaurusConfig();

    // Check 10: Build Readiness
    await this.checkBuildReadiness();

    this.printResults();
    return this.failed === 0;
  }

  async checkDirectoryStructure() {
    const name = 'Directory Structure';
    const requiredDirs = [
      'docs',
      'docs/modules',
      'docs/modules/week-01-02-fundamentals',
      'docs/modules/week-03-05-ros2',
      'docs/modules/week-06-07-simulation',
      'docs/modules/week-13-conversational',
      'docs/capstone',
      'src',
      'src/services',
      'assets',
      'assets/code-examples',
      'assets/code-examples/ros2',
      'assets/code-examples/gazebo',
      'assets/code-examples/unity',
      'assets/code-examples/isaac',
      'assets/images'
    ];

    let passed = true;
    const errors = [];

    for (const dir of requiredDirs) {
      const fullPath = path.join(process.cwd(), dir);
      if (!fs.existsSync(fullPath)) {
        passed = false;
        errors.push(`Missing directory: ${dir}`);
      }
    }

    this.addResult(name, passed, errors);
  }

  async checkConfigFiles() {
    const name = 'Configuration Files';
    const requiredFiles = [
      'docusaurus.config.js',
      'package.json',
      'sidebars.js',
      '.gitignore',
      'README.md'
    ];

    let passed = true;
    const errors = [];

    for (const file of requiredFiles) {
      const fullPath = path.join(process.cwd(), file);
      if (!fs.existsSync(fullPath)) {
        passed = false;
        errors.push(`Missing configuration file: ${file}`);
      }
    }

    this.addResult(name, passed, errors);
  }

  async checkDocumentationModules() {
    const name = 'Documentation Modules';
    const requiredDocs = [
      'docs/modules/week-01-02-fundamentals/physical-ai-principles.md',
      'docs/modules/week-01-02-fundamentals/embodied-intelligence.md',
      'docs/modules/week-03-05-ros2/ros2-architecture.md',
      'docs/modules/week-03-05-ros2/nodes-topics-services.md',
      'docs/modules/week-03-05-ros2/python-ros-packages.md',
      'docs/modules/week-06-07-simulation/gazebo-setup.md',
      'docs/modules/week-06-07-simulation/unity-visualization.md',
      'docs/modules/week-06-07-simulation/sensor-simulation.md',
      'docs/modules/week-13-conversational/language-models-robotics.md',
      'docs/modules/week-13-conversational/speech-recognition.md',
      'docs/modules/week-13-conversational/multi-modal-interaction.md'
    ];

    let passed = true;
    const errors = [];

    for (const doc of requiredDocs) {
      const fullPath = path.join(process.cwd(), doc);
      if (!fs.existsSync(fullPath)) {
        passed = false;
        errors.push(`Missing documentation: ${doc}`);
      } else {
        // Check if file has content
        const content = fs.readFileSync(fullPath, 'utf8');
        if (content.trim().length < 50) {
          errors.push(`Documentation file too short: ${doc}`);
          passed = false;
        }
      }
    }

    this.addResult(name, passed, errors);
  }

  async checkCodeExamples() {
    const name = 'Code Examples';
    const requiredExamples = [
      'assets/code-examples/ros2/simple_publisher.py',
      'assets/code-examples/ros2/simple_subscriber.py',
      'assets/code-examples/ros2/simple_service.py',
      'assets/code-examples/gazebo/robotics_lab.sdf',
      'assets/code-examples/gazebo/sensor_simulation.py',
      'assets/code-examples/gazebo/sensor_validator.py',
      'assets/code-examples/unity/UnityRobotController.cs',
      'assets/code-examples/unity/UnitySceneSetup.cs',
      'assets/code-examples/isaac/vla_pipeline.py',
      'assets/code-examples/isaac/voice_command_processing.py'
    ];

    let passed = true;
    const errors = [];

    for (const example of requiredExamples) {
      const fullPath = path.join(process.cwd(), example);
      if (!fs.existsSync(fullPath)) {
        passed = false;
        errors.push(`Missing code example: ${example}`);
      } else {
        // Check if file has content
        const content = fs.readFileSync(fullPath, 'utf8');
        if (content.trim().length < 20) {
          errors.push(`Code example file too short: ${example}`);
          passed = false;
        }
      }
    }

    this.addResult(name, passed, errors);
  }

  async checkServiceFiles() {
    const name = 'Service Files';
    const requiredServices = [
      'src/services/assessment-validator.ts',
      'src/services/code-executor.ts',
      'src/services/sensor-validator.ts',
      'src/services/physics-sim-validator.ts',
      'src/services/gemini-integration.ts',
      'src/services/gemini-api.test.ts',
      'src/services/vla-validator.ts'
    ];

    let passed = true;
    const errors = [];

    for (const service of requiredServices) {
      const fullPath = path.join(process.cwd(), service);
      if (!fs.existsSync(fullPath)) {
        passed = false;
        errors.push(`Missing service file: ${service}`);
      } else {
        // Check if file has content
        const content = fs.readFileSync(fullPath, 'utf8');
        if (content.trim().length < 50) {
          errors.push(`Service file too short: ${service}`);
          passed = false;
        }
      }
    }

    this.addResult(name, passed, errors);
  }

  async checkAssets() {
    const name = 'Assets';
    const requiredAssets = [
      'assets/images/',
      'docusaurus.config.js' // Check for theme configuration
    ];

    let passed = true;
    const errors = [];

    for (const asset of requiredAssets) {
      const fullPath = path.join(process.cwd(), asset);
      if (!fs.existsSync(fullPath)) {
        passed = false;
        errors.push(`Missing asset: ${asset}`);
      }
    }

    // Check for custom CSS that implements Purple + Neon theme
    const customCssPath = path.join(process.cwd(), 'src', 'css', 'custom.css');
    if (fs.existsSync(customCssPath)) {
      const cssContent = fs.readFileSync(customCssPath, 'utf8');
      // Check for purple colors in the CSS
      const hasPurple = cssContent.includes('#8a2be2') || cssContent.toLowerCase().includes('purple');
      // Check for neon colors in the CSS
      const hasNeon = cssContent.includes('#39ff14') || cssContent.toLowerCase().includes('neon');

      if (!hasPurple || !hasNeon) {
        errors.push('Custom CSS may not have Purple + Neon theme properly configured');
        passed = false;
      }
    } else {
      errors.push('Custom CSS file (src/css/custom.css) not found for theme configuration');
      passed = false;
    }

    this.addResult(name, passed, errors);
  }

  async checkEnvironmentVariables() {
    const name = 'Environment Variables';
    let passed = true;
    const errors = [];

    // Check if .env file exists or if required env vars are set
    const envFile = path.join(process.cwd(), '.env');
    if (!fs.existsSync(envFile)) {
      // Check if environment variables are set in the system
      if (!process.env.GOOGLE_API_KEY) {
        errors.push('GOOGLE_API_KEY environment variable not set (required for Gemini integration)');
        passed = false;
      }
    }

    this.addResult(name, passed, errors);
  }

  async checkDependencies() {
    const name = 'Dependencies';
    let passed = true;
    const errors = [];

    if (fs.existsSync(path.join(process.cwd(), 'package.json'))) {
      try {
        const packageJson = JSON.parse(fs.readFileSync(path.join(process.cwd(), 'package.json'), 'utf8'));

        // Check for required dependencies
        const requiredDeps = ['@docusaurus/core', '@docusaurus/preset-classic'];
        for (const dep of requiredDeps) {
          if (!packageJson.dependencies?.[dep] && !packageJson.devDependencies?.[dep]) {
            errors.push(`Missing required dependency: ${dep}`);
            passed = false;
          }
        }
      } catch (e) {
        errors.push('Invalid package.json file');
        passed = false;
      }
    } else {
      errors.push('package.json file not found');
      passed = false;
    }

    this.addResult(name, passed, errors);
  }

  async checkDocusaurusConfig() {
    const name = 'Docusaurus Configuration';
    let passed = true;
    const errors = [];

    if (fs.existsSync(path.join(process.cwd(), 'docusaurus.config.js'))) {
      const configContent = fs.readFileSync(path.join(process.cwd(), 'docusaurus.config.js'), 'utf8');

      // Check for essential configuration
      if (!configContent.includes('title') || !configContent.includes('Physical AI')) {
        errors.push('Docusaurus config missing title');
        passed = false;
      }

      if (!configContent.includes('preset') || !configContent.includes('classic')) {
        errors.push('Docusaurus config missing classic preset');
        passed = false;
      }
    } else {
      errors.push('docusaurus.config.js file not found');
      passed = false;
    }

    this.addResult(name, passed, errors);
  }

  async checkBuildReadiness() {
    const name = 'Build Readiness';
    let passed = true;
    const errors = [];

    // Check if build command would likely succeed
    const requiredFilesForBuild = [
      'package.json',
      'docusaurus.config.js',
      'sidebars.js',
      'src/',
      'docs/'
    ];

    for (const file of requiredFilesForBuild) {
      const fullPath = path.join(process.cwd(), file);
      if (!fs.existsSync(fullPath)) {
        errors.push(`Missing required file for build: ${file}`);
        passed = false;
      }
    }

    this.addResult(name, passed, errors);
  }

  addResult(name, passed, errors = []) {
    this.results.push({
      name,
      passed,
      errors
    });

    if (passed) {
      this.passed++;
    } else {
      this.failed++;
    }
  }

  printResults() {
    console.log('\n📊 Health Check Results:');
    console.log('=======================');

    for (const result of this.results) {
      const status = result.passed ? '✅ PASS' : '❌ FAIL';
      console.log(`\n${status} ${result.name}`);

      if (result.errors.length > 0) {
        for (const error of result.errors) {
          console.log(`   • ${error}`);
        }
      }
    }

    console.log('\n📈 Summary:');
    console.log(`   Total Checks: ${this.results.length}`);
    console.log(`   Passed: ${this.passed}`);
    console.log(`   Failed: ${this.failed}`);

    if (this.failed === 0) {
      console.log('\n🎉 All health checks passed! The system is ready for production.');
    } else {
      console.log(`\n⚠️  ${this.failed} check(s) failed. Please address the issues above before deployment.`);
    }
  }

  async checkExternalResources() {
    // This would check external resources like API endpoints
    // For now, just a placeholder
  }
}

// Run health check when executed directly
if (require.main === module) {
  const healthChecker = new HealthChecker();

  healthChecker.runAllChecks()
    .then(success => {
      process.exit(success ? 0 : 1);
    })
    .catch(error => {
      console.error('Health check failed with error:', error);
      process.exit(1);
    });
}

module.exports = HealthChecker;