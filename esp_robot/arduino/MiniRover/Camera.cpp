#include "Camera.h"

Camera::Camera()
{

}

Camera::~Camera()
{
}

void Camera::initialize()
{
	camera_config_t config;
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_d0 = Y2_GPIO_NUM;
	config.pin_d1 = Y3_GPIO_NUM;
	config.pin_d2 = Y4_GPIO_NUM;
	config.pin_d3 = Y5_GPIO_NUM;
	config.pin_d4 = Y6_GPIO_NUM;
	config.pin_d5 = Y7_GPIO_NUM;
	config.pin_d6 = Y8_GPIO_NUM;
	config.pin_d7 = Y9_GPIO_NUM;
	config.pin_xclk = XCLK_GPIO_NUM;
	config.pin_pclk = PCLK_GPIO_NUM;
	config.pin_vsync = VSYNC_GPIO_NUM;
	config.pin_href = HREF_GPIO_NUM;
	config.pin_sscb_sda = SIOD_GPIO_NUM;
	config.pin_sscb_scl = SIOC_GPIO_NUM;
	config.pin_pwdn = PWDN_GPIO_NUM;
	config.pin_reset = RESET_GPIO_NUM;
	config.xclk_freq_hz = 20000000;
	config.pixel_format = PIXFORMAT_JPEG;
	//init with high specs to pre-allocate larger buffers
	if (psramFound())
	{
		config.frame_size = FRAMESIZE_UXGA;
		config.jpeg_quality = 10;
		config.fb_count = 2;
	}
	else
	{
		config.frame_size = FRAMESIZE_SVGA;
		config.jpeg_quality = 12;
		config.fb_count = 1;
	}


	// camera init
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK)
	{
		Serial.printf("Camera init failed with error 0x%x", err);
		return;
	}

	sensor_t* s = esp_camera_sensor_get();
	//initial sensors are flipped vertically and colors are a bit saturated
	if (s->id.PID == OV3660_PID)
	{
		s->set_vflip(s, 1);//flip it back
		s->set_brightness(s, 1);//up the blightness just a bit
		s->set_saturation(s, -2);//lower the saturation
	}
	//drop down frame size for higher initial frame rate
	s->set_framesize(s, FRAMESIZE_QVGA);


	s->set_vflip(s, 1);
	//s->set_hmirror(s, 1);
}

void Camera::startCameraServer()
{
	startCameraServer_impl();
}



#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static ra_filter_t* ra_filter_init(ra_filter_t* filter, size_t sample_size)
{
	memset(filter, 0, sizeof(ra_filter_t));

	filter->values = (int*)malloc(sample_size * sizeof(int));
	if (!filter->values)
	{
		return NULL;
	}
	memset(filter->values, 0, sample_size * sizeof(int));

	filter->size = sample_size;
	return filter;
}

static int ra_filter_run(ra_filter_t* filter, int value)
{
	if (!filter->values)
	{
		return value;
	}
	filter->sum -= filter->values[filter->index];
	filter->values[filter->index] = value;
	filter->sum += filter->values[filter->index];
	filter->index++;
	filter->index = filter->index % filter->size;
	if (filter->count < filter->size)
	{
		filter->count++;
	}
	return filter->sum / filter->count;
}


static esp_err_t stream_handler(httpd_req_t* req)
{
	camera_fb_t* fb = NULL;
	esp_err_t res = ESP_OK;
	size_t _jpg_buf_len = 0;
	uint8_t* _jpg_buf = NULL;
	char* part_buf[64];
	dl_matrix3du_t* image_matrix = NULL;

	static int64_t last_frame = 0;
	if (!last_frame)
	{
		last_frame = esp_timer_get_time();
	}

	res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
	if (res != ESP_OK)
	{
		return res;
	}

	while (true)
	{
		fb = esp_camera_fb_get();
		if (!fb)
		{
			Serial.println("Camera capture failed");
			res = ESP_FAIL;
		}
		else
		{
				_jpg_buf_len = fb->len;
				_jpg_buf = fb->buf;
		}
		if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
		if (fb)
		{
			esp_camera_fb_return(fb);
			fb = NULL;
			_jpg_buf = NULL;
		}
		else if (_jpg_buf)
		{
			free(_jpg_buf);
			_jpg_buf = NULL;
		}
		if (res != ESP_OK)
		{
			break;
		}
		int64_t fr_end = esp_timer_get_time();
		int64_t frame_time = fr_end - last_frame;
		last_frame = fr_end;
		frame_time /= 1000;
		uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
		Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)\n",
			(uint32_t)(_jpg_buf_len),
			(uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
			avg_frame_time, 1000.0 / avg_frame_time);
	}

	last_frame = 0;
	return res;
}


static void startCameraServer_impl()
{
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	//httpd_uri_t index_uri = {
	//	.uri = "/",
	//	.method = HTTP_GET,
	//	.handler = index_handler,
	//	.user_ctx = NULL
	//};

	//httpd_uri_t status_uri = {
	//	.uri = "/status",
	//	.method = HTTP_GET,
	//	.handler = status_handler,
	//	.user_ctx = NULL
	//};

	//httpd_uri_t cmd_uri = {
	//	.uri = "/control",
	//	.method = HTTP_GET,
	//	.handler = cmd_handler,
	//	.user_ctx = NULL
	//};

	httpd_uri_t stream_uri = {
		 .uri = "/",
		 .method = HTTP_GET,
		 .handler = stream_handler,
		 .user_ctx = NULL
	};


	ra_filter_init(&ra_filter, 20);


	//Serial.printf("Starting web server on port: '%d'\n", config.server_port);
	//if (httpd_start(&camera_httpd, &config) == ESP_OK)
	//{
	//	httpd_register_uri_handler(camera_httpd, &index_uri);
	//	httpd_register_uri_handler(camera_httpd, &cmd_uri);
	//	httpd_register_uri_handler(camera_httpd, &status_uri);
	//}

	//config.server_port += 1;
	//config.ctrl_port += 1;
	Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
	//Serial.printf("Starting control server on port: '%d'\n", config.ctrl_port);


	if (httpd_start(&stream_httpd, &config) == ESP_OK)
	{
		httpd_register_uri_handler(stream_httpd, &stream_uri);
	}
}
