---
title: Understanding ROS 2
sidebar_position: 1
---

# Understanding ROS 2

## Overview

The Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools and libraries that help you build robot applications across a wide variety of robotic platforms.

## Key Concepts

### Nodes
Nodes are processes that perform computation. ROS 2 graphs are composed of multiple nodes working in concert.

### Topics
Topics are named buses over which nodes exchange messages. Topics are one of the main ways that data moves around a ROS 2 system.

### Services
Services allow nodes to send a request and receive a response.

### Actions
Actions are a form of asynchronous RPC that allow a node to invoke an action on another node and receive feedback and a result.

## Why ROS 2?

- **Distributed**: Runs on multiple computers and devices
- **Language agnostic**: Supports Python, C++, and other languages
- **Middleware agnostic**: Works with different communication middleware
- **Real-time capable**: Can be used in real-time systems
- **Security**: Built-in security features

## Getting Started

To begin working with ROS 2, you'll need to install it on your system and understand the basic concepts of nodes, topics, and services.

## Next Steps

Learn about [Working with Nodes](working-with-nodes) to understand how to create and manage ROS 2 nodes in your robotics applications.