# Data Model: Physical AI & Humanoid Robotics Book Outline

This document defines the key entities and their attributes for the "Physical AI & Humanoid Robotics Book" outline, based on the functional requirements and the objective of creating a Docusaurus-compatible structure.

## Entities

### Module
Represents a top-level organizational unit of the book, containing a group of related chapters.
-   **Attributes**:
    -   `name`: A string, representing the action-oriented title of the module (e.g., "Mastering ROS 2 Fundamentals").
    -   `slug`: A string, a URL-friendly identifier for the module, derived from the name (e.g., "ros2-fundamentals").
    -   `order`: An integer, defining the display order of the module in the book.
-   **Relationships**:
    -   Contains one-to-many `Chapter` entities.

### Chapter
Represents a sub-unit within a module, focusing on a specific topic.
-   **Attributes**:
    -   `title`: A string, representing the title of the chapter.
    -   `slug`: A string, a URL-friendly identifier for the chapter, derived from the title.
    -   `description`: A string, a concise, one-line summary of the chapter's content.
    -   `order`: An integer, defining the display order of the chapter within its module.
    -   `file_path`: A string, representing the relative path to the Markdown file for the chapter (e.g., `docs/ros2-fundamentals/understanding-nodes.md`).
-   **Relationships**:
    -   Belongs to one `Module` entity.

### Description
A concise, one-line summary of a chapter's content.
-   **Attributes**:
    -   `content`: A string, the actual one-line descriptive text.
-   **Relationships**:
    -   Associated with one `Chapter` entity.
