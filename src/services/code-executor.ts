/**
 * Automated code execution test framework for Physical AI & Humanoid Robotics Book
 * Provides safe execution environment for student code examples with validation
 */

interface CodeExecutionRequest {
  code: string;
  language: 'python' | 'typescript' | 'javascript' | 'bash';
  inputs?: string[];
  timeout?: number; // in milliseconds
  expectedOutput?: string;
}

interface CodeExecutionResult {
  success: boolean;
  output: string;
  error?: string;
  executionTime: number;
  matchesExpected?: boolean;
  passed?: boolean;
}

class CodeExecutor {
  private defaultTimeout: number = 5000; // 5 seconds default timeout

  constructor(timeout?: number) {
    if (timeout) {
      this.defaultTimeout = timeout;
    }
  }

  /**
   * Execute code in a safe environment
   * Note: In a real implementation, this would use a secure sandbox environment
   * For this example, we'll simulate the execution
   */
  public async executeCode(request: CodeExecutionRequest): Promise<CodeExecutionResult> {
    const startTime = Date.now();
    const timeout = request.timeout || this.defaultTimeout;

    // Validate language support
    if (!this.isSupportedLanguage(request.language)) {
      return {
        success: false,
        output: '',
        error: `Language ${request.language} is not supported`,
        executionTime: 0
      };
    }

    try {
      // Simulate code execution based on language
      const result = await this.simulateCodeExecution(request);

      const executionTime = Date.now() - startTime;

      // Check if execution exceeded timeout
      if (executionTime > timeout) {
        return {
          success: false,
          output: '',
          error: `Execution timed out after ${timeout}ms`,
          executionTime
        };
      }

      // If expected output is provided, check if it matches
      if (request.expectedOutput !== undefined) {
        result.matchesExpected = result.output.includes(request.expectedOutput);
        result.passed = result.matchesExpected;
      }

      result.executionTime = executionTime;

      return result;
    } catch (error: any) {
      return {
        success: false,
        output: '',
        error: error.message || 'Unknown error during execution',
        executionTime: Date.now() - startTime
      };
    }
  }

  /**
   * Simulate code execution for different languages
   * In a real implementation, this would actually execute the code in a sandbox
   */
  private async simulateCodeExecution(request: CodeExecutionRequest): Promise<CodeExecutionResult> {
    // This is a simplified simulation - in reality, this would execute the code
    // in a secure sandbox environment to prevent malicious code execution
    const code = request.code.toLowerCase();

    // Simulate different types of code execution
    if (request.language === 'python') {
      if (code.includes('ros2') || code.includes('rclpy')) {
        return {
          success: true,
          output: 'ROS 2 node initialized successfully\nPublisher and subscriber created\nNode spinning...'
        };
      } else if (code.includes('gazebo') || code.includes('simulation')) {
        return {
          success: true,
          output: 'Gazebo simulation started\nRobot model loaded\nPhysics engine running...'
        };
      } else if (code.includes('print') || code.includes('hello')) {
        return {
          success: true,
          output: 'Hello, Physical AI World!\nExecution completed successfully'
        };
      }
    } else if (request.language === 'typescript' || request.language === 'javascript') {
      if (code.includes('ros2') || code.includes('ros')) {
        return {
          success: true,
          output: 'ROS 2 TypeScript client initialized\nConnection established\nReady to publish/subscribe'
        };
      } else if (code.includes('isaac') || code.includes('nvidia')) {
        return {
          success: true,
          output: 'Isaac simulation environment loaded\nRobot control interface ready'
        };
      } else {
        return {
          success: true,
          output: 'JavaScript/TypeScript code executed successfully'
        };
      }
    } else if (request.language === 'bash') {
      if (code.includes('ros2')) {
        return {
          success: true,
          output: 'ROS 2 environment sourced\nAvailable commands: ros2 run, ros2 topic, ros2 service...'
        };
      } else {
        return {
          success: true,
          output: 'Bash script executed successfully'
        };
      }
    }

    // Default response if no specific simulation is found
    return {
      success: true,
      output: `Code executed successfully in ${request.language} environment`
    };
  }

  /**
   * Validate if the language is supported
   */
  private isSupportedLanguage(language: string): language is 'python' | 'typescript' | 'javascript' | 'bash' {
    return ['python', 'typescript', 'javascript', 'bash'].includes(language);
  }

  /**
   * Execute and validate a student's code submission against requirements
   */
  public async validateStudentCode(code: string, language: 'python' | 'typescript' | 'javascript' | 'bash', requirements: string[]): Promise<CodeExecutionResult> {
    // First, execute the code
    const executionResult = await this.executeCode({
      code,
      language,
      timeout: 10000 // 10 second timeout for student submissions
    });

    if (!executionResult.success) {
      return executionResult;
    }

    // Check if the code meets the requirements
    let meetsRequirements = true;
    const missingRequirements: string[] = [];

    for (const requirement of requirements) {
      if (!code.toLowerCase().includes(requirement.toLowerCase())) {
        meetsRequirements = false;
        missingRequirements.push(requirement);
      }
    }

    if (!meetsRequirements) {
      executionResult.success = false;
      executionResult.error = `Code is missing required elements: ${missingRequirements.join(', ')}`;
    } else {
      executionResult.passed = true;
      executionResult.output += `\nCode meets all requirements: ${requirements.join(', ')}`;
    }

    return executionResult;
  }

  /**
   * Run a set of test cases against the provided code
   */
  public async runTestCases(code: string, language: 'python' | 'typescript' | 'javascript' | 'bash', testCases: Array<{input: string, expectedOutput: string}>): Promise<{allPassed: boolean, results: CodeExecutionResult[]}> {
    const results: CodeExecutionResult[] = [];
    let allPassed = true;

    for (const testCase of testCases) {
      // In a real implementation, we would inject the test input into the code
      // For this simulation, we'll just check if the expected output appears
      const result = await this.executeCode({
        code,
        language,
        expectedOutput: testCase.expectedOutput
      });

      results.push(result);
      if (!result.passed) {
        allPassed = false;
      }
    }

    return { allPassed, results };
  }
}

// Singleton instance
const codeExecutor = new CodeExecutor();

export { CodeExecutor, CodeExecutionRequest, CodeExecutionResult };
export default codeExecutor;