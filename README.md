
```
openssl req -x509 -nodes -subj /CN=esp32.example.com -days 3650 -newkey rsa:2048 -keyout esp32.example.com.key -out esp32.example.com.crt
curl -v -F "url=https://esp32.example.com:8443/198/ESP32_LYWSD03MMC" -F "certificate=@esp32.example.com.crt" https://api.telegram.org/bot123456789:XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX/setWebhook
```

```
server {
    listen 8443 ssl http2 default_server;
    server_name esp32.example.com;
    
    ssl_certificate /etc/ssl/certs/esp32.example.com.crt;
    ssl_certificate_key /etc/ssl/private/esp32.example.com.key;
    
    allow 149.154.160.0/20;
    allow 91.108.4.0/22;
    deny all;

    location ~* ^/(?<ip>(198|243))/ESP32_LYWSD03MMC$ {
        rewrite /([^./]+)/(.*) /$2 break;
        proxy_pass http://192.168.1.${ip}:80;
    }
}

server {
    listen 8888 default_server;
    server_name api.telegram.org;
    
    allow 192.168.1.0/24;
    deny all;

    location / {
        proxy_pass https://api.telegram.org:443;
    }
}

```