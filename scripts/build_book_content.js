const fs = require('fs');
const path = require('path');

// Function to recursively read markdown files from a directory
function readMarkdownFiles(dir, filelist = []) {
  const files = fs.readdirSync(dir);
  
  files.forEach(file => {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);
    
    if (stat.isDirectory()) {
      filelist = readMarkdownFiles(filePath, filelist);
    } else if (path.extname(file) === '.md') {
      filelist.push(filePath);
    }
  });
  
  return filelist;
}

// Read all markdown files from the docs directory
const docsDir = path.join(__dirname, '..', 'docs');
const markdownFiles = readMarkdownFiles(docsDir);

let bookContent = "# Physical AI & Humanoid Robotics Book\n\n";

// Add content from each markdown file
markdownFiles.forEach(file => {
  const content = fs.readFileSync(file, 'utf8');
  
  // Remove frontmatter if present (common in Docusaurus)
  const contentWithoutFrontmatter = content.replace(/^---\n.*?\n---\n/s, '');
  
  bookContent += `\n${contentWithoutFrontmatter}\n`;
});

// Create static directory if it doesn't exist
const staticDir = path.join(__dirname, '..', 'static');
if (!fs.existsSync(staticDir)) {
  fs.mkdirSync(staticDir);
}

// Write the combined content to a file in the static directory
const outputPath = path.join(staticDir, 'book-content.md');
fs.writeFileSync(outputPath, bookContent);

console.log(`Successfully created book content file with ${markdownFiles.length} source files`);
console.log(`Total content length: ${bookContent.length} characters`);
console.log(`Output path: ${outputPath}`);