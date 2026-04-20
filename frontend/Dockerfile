# Build
FROM node:20-alpine AS build
WORKDIR /app

COPY package.json tsconfig.json vite.config.ts index.html ./
COPY public ./public
COPY src ./src

RUN npm install
RUN npm run build

# Serve
FROM nginx:alpine
COPY nginx.conf /etc/nginx/conf.d/default.conf
COPY --from=build /app/dist /usr/share/nginx/html
