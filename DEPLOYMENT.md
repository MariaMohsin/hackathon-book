# Deployment Guide for RAG Chatbot

## Prerequisites
- Docker and Docker Compose installed
- API keys for Cohere, Neon Postgres, and Qdrant
- Server with at least 2GB RAM and 5GB disk space

## Environment Setup

### 1. Create Environment File
Create a `.env` file based on the `.env.example` with your actual API keys:

```bash
cp .env.example .env
```

Then edit the `.env` file with your actual API keys:

```env
COHERE_API_KEY=your_actual_cohere_api_key
NEON_DATABASE_URL=your_actual_neon_database_url
QDRANT_URL=your_actual_qdrant_url
QDRANT_API_KEY=your_actual_qdrant_api_key
```

### 2. Configure Application Settings
In your `.env` file, you can also configure:
- `DEBUG`: Set to `True` for development, `False` for production
- `LOG_LEVEL`: Set to `INFO`, `DEBUG`, `WARNING`, etc.
- `ENVIRONMENT`: Set to `development`, `staging`, or `production`

## Deployment Options

### Option 1: Docker Compose (Recommended)

1. Build and start the services:
```bash
docker-compose up -d
```

2. Check the logs to ensure everything is running:
```bash
docker-compose logs -f
```

3. The API will be available at `http://your-server-ip:8000`

4. To stop the services:
```bash
docker-compose down
```

### Option 2: Direct Docker

1. Build the image:
```bash
docker build -t rag-chatbot .
```

2. Run the container:
```bash
docker run -d \
  --name rag-chatbot \
  -p 8000:8000 \
  --env-file .env \
  rag-chatbot
```

### Option 3: Direct Python (for development)

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Run the application:
```bash
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## Configuration for Production

### 1. SSL/TLS
For production, it's recommended to use a reverse proxy (like Nginx) with SSL/TLS certificates.

Example Nginx configuration:
```nginx
server {
    listen 443 ssl;
    server_name your-domain.com;

    ssl_certificate /path/to/certificate.crt;
    ssl_certificate_key /path/to/private.key;

    location / {
        proxy_pass http://localhost:8000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

### 2. Environment Variables
Ensure your production environment has appropriate values for:
- `DEBUG=False`
- `LOG_LEVEL=INFO`
- `ENVIRONMENT=production`

### 3. Scaling
The application supports horizontal scaling. For production, consider using a container orchestration platform like Kubernetes or a platform like Docker Swarm.

Example for running multiple instances with Docker Compose:
```yaml
services:
  rag-chatbot:
    build: .
    deploy:
      replicas: 3
    ports:
      - "8000:8000"
    # ... rest of configuration
```

## Health Checks and Monitoring

### Health Endpoints
- `/api/health` - Basic health check
- `/api/health/extended` - Detailed system and service status
- `/api/health/readiness` - Readiness for traffic

### Logs
Docker logs can be accessed with:
```bash
docker-compose logs -f rag-chatbot
```

For persistent logging, consider using a centralized logging solution.

## Security Considerations

### 1. API Key Security
- Never commit API keys to version control
- Use environment variables for all sensitive credentials
- Rotate API keys regularly

### 2. Rate Limiting
- The application includes built-in rate limiting (50 requests/hour/IP)
- Adjust limits as needed for your use case

### 3. Network Security
- Use SSL/TLS for all production traffic
- Consider using a Web Application Firewall (WAF)

## Backup and Recovery

### Database Backup
Regularly backup your Neon Postgres database:
```bash
pg_dump $NEON_DATABASE_URL > backup.sql
```

### Data Recovery
To restore from backup:
```bash
psql $NEON_DATABASE_URL < backup.sql
```

## Troubleshooting

### Common Issues

1. **Connection Errors**: Verify that your API keys and URLs are correct and that your server can reach the external services.

2. **Performance Issues**: Monitor your Qdrant Cloud and Neon Postgres usage to ensure you're within your plan limits.

3. **Memory Issues**: If you're experiencing memory issues, consider increasing the system resources or optimizing your document chunking strategy.

4. **Docker Issues**: Check Docker logs with `docker logs rag-chatbot` for more details.

### Getting Help
- Check the application logs for error details
- Verify all environment variables are properly set
- Ensure your external service accounts (Cohere, Qdrant, Neon) are active and in good standing

## Updates

To update to a new version:
1. Pull the latest code
2. Update your `requirements.txt` if needed
3. Rebuild the Docker image: `docker-compose build`
4. Restart the services: `docker-compose up -d`