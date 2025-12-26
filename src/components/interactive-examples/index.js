import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Example interactive component for code execution
const InteractiveCodeExample = ({ code, language = 'typescript', title = 'Code Example' }) => {
  return (
    <div className="container margin-vert--md">
      <div className="row">
        <div className="col col--12">
          <h3>{title}</h3>
          <BrowserOnly>
            {() => {
              const { default: CodeBlock } = require('@theme/CodeBlock');
              return <CodeBlock language={language}>{code}</CodeBlock>;
            }}
          </BrowserOnly>
        </div>
      </div>
    </div>
  );
};

// Example interactive diagram component
const InteractiveDiagram = ({ title = 'System Diagram', description = 'Interactive diagram showing system components' }) => {
  return (
    <div className="container margin-vert--md">
      <div className="row">
        <div className="col col--12">
          <h3>{title}</h3>
          <div className="neon-border" style={{ padding: '20px', borderRadius: '8px', backgroundColor: 'var(--ifm-color-emphasis-100)' }}>
            <p>{description}</p>
            <div style={{ textAlign: 'center', marginTop: '10px' }}>
              <span className="neon-text">Interactive Diagram Placeholder</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export { InteractiveCodeExample, InteractiveDiagram };