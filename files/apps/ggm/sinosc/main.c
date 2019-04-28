//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/pcm.h>

//-----------------------------------------------------------------------------

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
#error "CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS not supported"
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION
#error "CONFIG_AUDIO_MULTI_SESSION not supported"
#endif

#define AUDIO_DEVICE "/dev/audio/pcm1"

//-----------------------------------------------------------------------------

struct player_s {
	int fd;			// file descriptor for audio device
	mqd_t mq;		// message queue for the playthread
	char mqname[16];	// name of message queue
	struct ap_buffer_s *pbuf[CONFIG_AUDIO_NUM_BUFFERS];	// audio buffers
};

#if 0

#define AUDIOIOC_GETCAPS            _AUDIOIOC(1)

#define AUDIOIOC_RESERVE            _AUDIOIOC(2)
#define AUDIOIOC_RELEASE            _AUDIOIOC(3)

#define AUDIOIOC_CONFIGURE          _AUDIOIOC(4)
#define AUDIOIOC_SHUTDOWN           _AUDIOIOC(5)

#define AUDIOIOC_START              _AUDIOIOC(6)
#define AUDIOIOC_STOP               _AUDIOIOC(7)
#define AUDIOIOC_PAUSE              _AUDIOIOC(8)
#define AUDIOIOC_RESUME             _AUDIOIOC(9)

#define AUDIOIOC_GETBUFFERINFO      _AUDIOIOC(10)
#define AUDIOIOC_SETBUFFERINFO      _AUDIOIOC(17)

#define AUDIOIOC_ALLOCBUFFER        _AUDIOIOC(11)
#define AUDIOIOC_FREEBUFFER         _AUDIOIOC(12)

#define AUDIOIOC_ENQUEUEBUFFER      _AUDIOIOC(13)

#define AUDIOIOC_REGISTERMQ         _AUDIOIOC(14)
#define AUDIOIOC_UNREGISTERMQ       _AUDIOIOC(15)

#define AUDIOIOC_HWRESET            _AUDIOIOC(16)

#endif

//-----------------------------------------------------------------------------

static void player_close(struct player_s *player) {

	DEBUGASSERT(player != NULL);

	// free buffers
	struct audio_buf_desc_s buf_desc;
	memset(&buf_desc, 0, sizeof(buf_desc));
	buf_desc.numbytes = CONFIG_AUDIO_BUFFER_NUMBYTES;
	audinfo("*** freeing audio buffers\n");
	for (int i = 0; i < CONFIG_AUDIO_NUM_BUFFERS; i++) {
		if (player->pbuf[i] != NULL) {
			buf_desc.u.pBuffer = player->pbuf[i];
			int rc = ioctl(player->fd, AUDIOIOC_FREEBUFFER, (unsigned long)&buf_desc);
			if (rc != sizeof(buf_desc)) {
				printf("ioctl(AUDIOIOC_FREEBUFFER) failed %d\n", rc);
			}
		}
	}

	// unregister the message queue
	if (player->fd > 0 && player->mq != NULL) {
		audinfo("*** ioctl AUDIOIOC_UNREGISTERMQ\n");
		ioctl(player->fd, AUDIOIOC_UNREGISTERMQ, (unsigned long)player->mq);
	}
	// close and unlink the message queue
	if (player->mq != NULL) {
		audinfo("*** closing message queue\n");
		mq_close(player->mq);
		mq_unlink(player->mqname);
		player->mq = NULL;
	}
	// release the device
	if (player->fd > 0) {
		audinfo("*** ioctl AUDIOIOC_RELEASE\n");
		ioctl(player->fd, AUDIOIOC_RELEASE, 0);
	}
	// close the device
	if (player->fd > 0) {
		audinfo("*** closing audio device\n");
		close(player->fd);
		player->fd = -1;
	}
}

static int player_open(struct player_s *player) {

	DEBUGASSERT(player != NULL);

	memset(player, 0, sizeof(struct player_s));

	// open the device
	audinfo("*** opening audio device\n");
	int fd = open(AUDIO_DEVICE, O_RDWR);
	if (fd < 0) {
		printf("open() failed %d\n", fd);
		goto exit;
	}
	DEBUGASSERT(fd != 0);
	player->fd = fd;

	// reserve the device
	audinfo("*** ioctl AUDIOIOC_RESERVE\n");
	int rc = ioctl(player->fd, AUDIOIOC_RESERVE, 0);
	if (rc < 0) {
		printf("ioctl(AUDIOIOC_RESERVE) failed %d\n", rc);
		goto exit;
	}
	// create the message queue
	struct mq_attr attr;
	attr.mq_maxmsg = 16;
	attr.mq_msgsize = sizeof(struct audio_msg_s);
	attr.mq_curmsgs = 0;
	attr.mq_flags = 0;
	snprintf(player->mqname, sizeof(player->mqname), "/tmp/%p", player);
	audinfo("*** opening message queue\n");
	player->mq = mq_open(player->mqname, O_RDWR | O_CREAT, 0644, &attr);
	if (player->mq == NULL) {
		printf("mq_open() failed\n");
		goto exit;
	}
	// register the message queue
	audinfo("*** ioctl AUDIOIOC_REGISTERMQ\n");
	rc = ioctl(player->fd, AUDIOIOC_REGISTERMQ, (unsigned long)player->mq);
	if (rc < 0) {
		printf("ioctl(AUDIOIOC_REGISTERMQ) failed %d\n", rc);
		goto exit;
	}
	// allocate buffers
	struct audio_buf_desc_s buf_desc;
	memset(&buf_desc, 0, sizeof(buf_desc));
	buf_desc.numbytes = CONFIG_AUDIO_BUFFER_NUMBYTES;
	audinfo("*** allocating audio buffers\n");
	for (int i = 0; i < CONFIG_AUDIO_NUM_BUFFERS; i++) {
		buf_desc.u.ppBuffer = &player->pbuf[i];
		rc = ioctl(player->fd, AUDIOIOC_ALLOCBUFFER, (unsigned long)&buf_desc);
		if (rc != sizeof(buf_desc)) {
			printf("ioctl(AUDIOIOC_ALLOCBUFFER) failed %d\n", rc);
			goto exit;
		}
	}

	return 0;

 exit:
	player_close(player);
	return -1;
}

//-----------------------------------------------------------------------------

static char *audio_caps_str(struct audio_caps_s *caps, char *str, size_t n) {
	int ofs = 0;
	ofs += scnprintf(&str[ofs], n - ofs, "ac_len %d\n", caps->ac_len);
	ofs += scnprintf(&str[ofs], n - ofs, "ac_type %d\n", caps->ac_type);
	ofs += scnprintf(&str[ofs], n - ofs, "ac_subtype %d\n", caps->ac_subtype);
	ofs += scnprintf(&str[ofs], n - ofs, "ac_channels %d\n", caps->ac_channels);
	ofs += scnprintf(&str[ofs], n - ofs, "ac_format.hw 0x%04x\n", caps->ac_format.hw);
	ofs += scnprintf(&str[ofs], n - ofs, "ac_controls.w 0x%08x\n", caps->ac_controls.w);
	return str;
}

static int get_caps(int fd) {
	struct audio_caps_s caps;
	int rc = 0;

	memset(&caps, 0, sizeof(caps));
	caps.ac_len = sizeof(caps);
	caps.ac_type = AUDIO_TYPE_QUERY;
	caps.ac_subtype = AUDIO_TYPE_QUERY;

	audinfo("*** ioctl AUDIOIOC_GETCAPS\n");
	rc = ioctl(fd, AUDIOIOC_GETCAPS, (unsigned long)&caps);
	if (rc != sizeof(caps)) {
		printf("ioctl() failed %d (%s %d)\n", rc, __FILE__, __LINE__);
		rc = -1;
		goto exit;
	}

	char tmp[256];
	printf("%s\n", audio_caps_str(&caps, tmp, sizeof(tmp)));

 exit:
	return rc;
}

//-----------------------------------------------------------------------------

static int set_volume(struct player_s *player, uint16_t volume) {

	struct audio_caps_desc_s cap_desc;
	memset(&cap_desc, 0, sizeof(cap_desc));
	cap_desc.caps.ac_len = sizeof(struct audio_caps_s);
	cap_desc.caps.ac_type = AUDIO_TYPE_FEATURE;
	cap_desc.caps.ac_format.hw = AUDIO_FU_VOLUME;
	cap_desc.caps.ac_controls.hw[0] = volume;

	audinfo("*** ioctl AUDIOIOC_CONFIGURE\n");
	int rc = ioctl(player->fd, AUDIOIOC_CONFIGURE, (unsigned long)&cap_desc);
	if (rc < 0) {
		printf("ioctl(AUDIOIOC_CONFIGURE) failed %d\n", rc);
		return -1;
	}
	return 0;
}

//-----------------------------------------------------------------------------

static int set_sampling(int fd, uint8_t bits_per_sample, uint16_t sample_rate) {
	struct audio_caps_desc_s cap_desc;
	int rc;

	memset(&cap_desc, 0, sizeof(cap_desc));
	cap_desc.caps.ac_len = sizeof(struct audio_caps_s);
	cap_desc.caps.ac_type = AUDIO_TYPE_OUTPUT;
	cap_desc.caps.ac_channels = 2;
	cap_desc.caps.ac_controls.hw[0] = sample_rate;
	cap_desc.caps.ac_controls.b[3] = sample_rate >> 16;
	cap_desc.caps.ac_controls.b[2] = bits_per_sample;

	audinfo("*** ioctl AUDIOIOC_CONFIGURE\n");
	rc = ioctl(fd, AUDIOIOC_CONFIGURE, (unsigned long)&cap_desc);
	if (rc < 0) {
		printf("ioctl() failed %d (%s %d)\n", rc, __FILE__, __LINE__);
		goto exit;
	}
 exit:
	return rc;
}

//-----------------------------------------------------------------------------

static int hw_reset(int fd) {
	int rc;

	audinfo("*** ioctl AUDIOIOC_HWRESET\n");
	rc = ioctl(fd, AUDIOIOC_HWRESET, 0);
	if (rc < 0) {
		printf("ioctl() failed %d (%s %d)\n", rc, __FILE__, __LINE__);
		rc = -1;
		goto exit;
	}

 exit:
	return rc;
}

//-----------------------------------------------------------------------------

static void fake_wav(struct ap_buffer_s *apb) {
	struct wav_header_s *wav = (struct wav_header_s *)&apb->samp[apb->curbyte];

	int rate = 48000;
	int bits_per_sample = 16;
	int channels = 2;

	wav->hdr.chunkid = WAV_HDR_CHUNKID;
	wav->hdr.format = WAV_HDR_FORMAT;
	wav->fmt.chunkid = WAV_FMT_CHUNKID;
	wav->fmt.chunklen = WAV_FMT_CHUNKLEN;
	wav->fmt.format = WAV_FMT_FORMAT;
	wav->fmt.samprate = rate;
	wav->fmt.byterate = (rate * channels * bits_per_sample) / 8;
	wav->fmt.nchannels = channels;
	wav->fmt.align = (channels * bits_per_sample) / 8;
	wav->fmt.bpsamp = 16;
	wav->data.chunkid = WAV_DATA_CHUNKID;
}

static void fill_buffer(struct ap_buffer_s *apb) {
	apb->nbytes = apb->nmaxbytes;
	apb->curbyte = 0;
	apb->flags = 0;
	for (int i = 0; i < apb->nbytes; i++) {
		apb->samp[i] = (uint8_t) rand();
	}
}

//-----------------------------------------------------------------------------

static int enqueue_buffer(int fd, struct ap_buffer_s *apb) {
	struct audio_buf_desc_s buf_desc;
	buf_desc.numbytes = apb->nbytes;
	buf_desc.u.pBuffer = apb;
	int rc = ioctl(fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long)&buf_desc);
	if (rc < 0) {
		printf("ioctl(AUDIOIOC_ENQUEUEBUFFER) failed %d\n", rc);
		return -1;
	}
	return 0;
}

//-----------------------------------------------------------------------------

static int play(struct player_s *player) {
	int rc = 0;

	// initial enqueue of buffers
	for (int i = 0; i < CONFIG_AUDIO_NUM_BUFFERS; i++) {
		// setup the buffer
		struct ap_buffer_s *apb = player->pbuf[i];
		apb->nbytes = apb->nmaxbytes;
		apb->curbyte = 0;
		apb->flags = 0;
		if (i == 0) {
			fake_wav(apb);
		}
		enqueue_buffer(player->fd, apb);
	}

	audinfo("*** audio start\n");
	rc = ioctl(player->fd, AUDIOIOC_START, 0);
	if (rc < 0) {
		printf("ioctl(AUDIOIOC_START) failed %d\n", rc);
		goto exit;
	}

	for (int i = 0; i < 128; i++) {
		// get a message
		struct audio_msg_s msg;
		ssize_t n = mq_receive(player->mq, (char *)&msg, sizeof(msg), NULL);
		if (n < 0) {
			printf("mq_receive() error %d\n", errno);
			continue;
		}
		switch (msg.msgId) {
		case AUDIO_MSG_DEQUEUE:
			DEBUGASSERT(msg.u.pPtr);
			printf("nq %d\n", i);
			fill_buffer(msg.u.pPtr);
			enqueue_buffer(player->fd, msg.u.pPtr);
			break;
		default:
			printf("unknown message %d\n", msg.msgId);
			break;
		}
	}

	audinfo("*** stop\n");
	rc = ioctl(player->fd, AUDIOIOC_STOP, 0);
	if (rc < 0) {
		printf("ioctl(AUDIOIOC_STOP) failed %d\n", rc);
		goto exit;
	}

 exit:
	return rc;
}

//-----------------------------------------------------------------------------

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int sinosc_main(int argc, char *argv[])
#endif
{
	struct player_s player;
	int rc;

	rc = player_open(&player);
	if (rc < 0) {
		printf("player_open() failed\n");
		return -1;
	}

	set_volume(&player, 200);

	play(&player);

	player_close(&player);

	return 0;
}

//-----------------------------------------------------------------------------
