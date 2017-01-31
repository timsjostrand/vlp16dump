/*
 * Velodyne VLP-16 packet parser and dumper.
 *
 * usage: vlp16dump [port]
 *
 * Author: Tim Sjöstrand <timjon@student.chalmers.se>
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <netdb.h>
#include <math.h>
#include <getopt.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define DEFAULT_PORT			2368
#define PACKET_SIZE				1206
#define DISTANCE_THRESHOLD		1.0f
#define DATA_BLOCK_SIZE			100
#define DATA_BLOCK_FLAG			(uint16_t) 0xFFEE
#define DATA_BLOCKS_COUNT		12
#define CHANNELS_COUNT			32
#define DISTANCE_UNIT_FRACTION	1000					/* meters */

#define bail(fmt, ...) do { fprintf(stderr, "ERROR: " fmt, ##__VA_ARGS__); exit(1); } while(0)
#define deg_to_rad(deg) (deg * M_PI / 180.0f)
#define rad_to_deg(rad) (rad * 180.0 / M_PI)

#define FACTORY_RETURN_MODE_STRONGEST	0x37
#define FACTORY_RETURN_MODE_LAST		0x38
#define FACTORY_RETURN_MODE_DUAL		0x39

#define FACTORY_SOURCE_HDL32E			0x21
#define FACTORY_SOURCE_VLP16			0x22

//#define DEBUG_FACTORY

#define SKIP_INVALID_DISTANCE
//#define AZIMUTH_NET_BYTE_ORDER

#define OUTPUT_FORMAT_DEBUG				0
#define OUTPUT_FORMAT_XYZ				1
#define OUTPUT_FORMAT_XYZ_BIN			2

/**
 * Statistics.
 */
struct statistics {
	uint32_t	packets_count;
	uint32_t	points_count;
	uint32_t	skipped_r;
	uint32_t	skipped_azimuth;
} statistics = { 0 };

struct __attribute__((__packed__)) vlp16_channel_data {
	uint16_t	distance;
	uint8_t		reflectivity;
};

struct __attribute__((__packed__)) vlp16_data_block {
	uint16_t					flag;
	uint16_t					azimuth;
	struct vlp16_channel_data	channels[CHANNELS_COUNT];
};

struct __attribute__((__packed__)) vlp16_packet {
	struct vlp16_data_block		blocks[DATA_BLOCKS_COUNT];
	uint32_t					timestamp;
	uint8_t						factory_return_mode;
	uint8_t						factory_source;
};

/**
 * Maps a laser ID to a vertical angle in degrees.
 */
static double vlp16_vertical_angles[] = {
	deg_to_rad(-15), // 0
	deg_to_rad(  1), // 1
	deg_to_rad(-13), // 2
	deg_to_rad( -3), // 3
	deg_to_rad(-11), // 4
	deg_to_rad(  5), // 5
	deg_to_rad( -9), // 6
	deg_to_rad(  7), // 7
	deg_to_rad( -7), // 8
	deg_to_rad(  9), // 9
	deg_to_rad( -5), // 10
	deg_to_rad( 11), // 11
	deg_to_rad( -3), // 12
	deg_to_rad( 13), // 13
	deg_to_rad( -1), // 14
	deg_to_rad( 15)  // 15
};

struct arguments {
	int		port;
	int		distance_min;
	int		distance_max;
	int		origo_x;
	int		origo_y;
	int		origo_z;
	int		packets_max;
	int		output_format;
} arguments;

struct arguments args_default = {
	.port = DEFAULT_PORT,
	.distance_min = 1000,
	.distance_max = -1,
	.origo_x = 0,
	.origo_y = 0,
	.origo_z = 0,
	.packets_max = -1,
	.output_format = OUTPUT_FORMAT_XYZ
};

static struct option long_options[] = {
	{ "port",			required_argument,	0, 'p' },
	{ "distance-min",	required_argument,	0, 'l' },
	{ "distance-max",	required_argument,	0, 'g' },
	{ "help",			no_argument,		0, 'h' },
	{ "origo-x",		required_argument,	0, 'x' },
	{ "origo-y",		required_argument,	0, 'y' },
	{ "origo-z",		required_argument,	0, 'z' },
	{ "packets-max",	required_argument,	0, 'c' },
	{ "output-format",	required_argument,	0, 'f' },
	{ 0 }
};

static void print_usage(int argc, char **argv)
{
	fprintf(stderr, "%s usage:\n", argv[0]);
	fprintf(stderr, "  --port          -p <PORT>    Listen port. Default: %d.\n", args_default.port);
	fprintf(stderr, "  --distance-min  -l <MM>      Discard distances less than this value, in mm. Default: %d.\n", args_default.distance_min);
	fprintf(stderr, "  --distance-max  -g <MM>      Discard distances greater than this value, in mm. Default: %d.\n", args_default.distance_max);
	fprintf(stderr, "  --origo-x       -x <MM>      Move origo X by this amount, in mm. Default: %d.\n", args_default.origo_x);
	fprintf(stderr, "  --origo-y       -y <MM>      Move origo Y by this amount, in mm. Default: %d.\n", args_default.origo_y);
	fprintf(stderr, "  --origo-z       -z <MM>      Move origo Z by this amount, in mm. Default: %d.\n", args_default.origo_z);
	fprintf(stderr, "  --packets-max   -c <COUNT>   Stop after receiving this many packets (-1 to disable). Default: %d.\n", args_default.packets_max);
	fprintf(stderr, "  --output-format -f <OUTFMT>  Specify output format. Default: xyz.\n");
	fprintf(stderr, "  --help          -h           Print this message.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Supported output formats:\n");
	fprintf(stderr, "  xyz (ASCII: x, y, z)\n");
	fprintf(stderr, "  debug (ASCII: x, y, z, reflectivity, distance, azimuth)\n");
}

static void parse_opts(int argc, char **argv)
{
	arguments = args_default;

	int opt = 0;
	int long_index = 0;
	while((opt = getopt_long(argc, argv, "p:l:g:hx:y:z:f:c:", long_options, &long_index)) != -1) {
		switch(opt) {
			case 'p':
				arguments.port = atoi(optarg);
				break;
			case 'l':
				arguments.distance_min = atoi(optarg);
				break;
			case 'g':
				arguments.distance_max = atoi(optarg);
				break;
			case 'x':
				arguments.origo_x = atoi(optarg);
				break;
			case 'y':
				arguments.origo_y = atoi(optarg);
				break;
			case 'z':
				arguments.origo_z = atoi(optarg);
				break;
			case 'c':
				arguments.packets_max = atoi(optarg);
				break;
			case 'f':
				if(strcmp(optarg, "xyz") == 0) {
					arguments.output_format = OUTPUT_FORMAT_XYZ;
				} else if(strcmp(optarg, "debug") == 0) {
					arguments.output_format = OUTPUT_FORMAT_DEBUG;
				} else if(strcmp(optarg, "xyzbin") == 0) {
					arguments.output_format = OUTPUT_FORMAT_XYZ_BIN;
				} else {
					fprintf(stderr, "Invalid output format: %s\n", optarg);
					print_usage(argc, argv);
					exit(EXIT_FAILURE);
				}
				break;
			case 'h':
			default:
				print_usage(argc, argv);
				exit(EXIT_FAILURE);
		}
	}
}

void hexdump(const char *title, const char *buf, const size_t count)
{
	fprintf(stderr, "%s: 0x", title);
	const unsigned char *p = (const unsigned char *) buf;
	for(int i=0; i < count; i++) {
		fprintf(stderr, "%02x", p[i]);
	}
	fprintf(stderr, "\n");
}

void hexdump_uint16(const char *title, const uint16_t i)
{
	hexdump(title, (char *) &i, sizeof(uint16_t));
}

/**
 * Convert spherical coordinates (r, omega, alpha) as reported by the VLP-16 to XYZ.
 *
 * r		Is the distance to the object.
 * omega	Is the vertical/elevation angle and is fixed based on the laser ID.
 * alpha	Is the horizontal angle/azimuth and is reported in every other data block.
 *
 * If r < 1 meter then that coordinate should be ignored.
 */
void vlp16_transform_coords(const int laser_id, const float alpha, const double distance, double *x, double *y, double *z)
{
	float omega = vlp16_vertical_angles[laser_id];
	*x = distance * cos(omega) * sin(alpha);
	*y = distance * cos(omega) * cos(alpha);
	*z = distance * sin(omega);
}

float vlp16_interpolate_azimuth(struct vlp16_packet *packet, int block_index)
{
	struct vlp16_data_block *this_block = &packet->blocks[block_index];
	uint16_t azimuth_min;
	uint16_t azimuth_max;

	/* There is no next-azimuth to compute delta from for the last block. Assume
	 * delta is constant and calculate it from the second-to-last instead. */
	if(block_index == DATA_BLOCKS_COUNT-1) {
		struct vlp16_data_block *prev_block = &packet->blocks[block_index - 1];
		azimuth_min = prev_block->azimuth % (360*100);
		azimuth_max = this_block->azimuth % (360*100);
	} else {
		struct vlp16_data_block *next_block = &packet->blocks[block_index + 1];
		azimuth_min = this_block->azimuth % (360*100);
		azimuth_max = next_block->azimuth % (360*100);
	}

	if(azimuth_max < azimuth_min) {
		azimuth_max += (360*100);
	}

	uint32_t interpolated = this_block->azimuth + (azimuth_max - azimuth_min) / 2;
	interpolated %= (360*100);

	return deg_to_rad(interpolated / 100.0f);
}

float vlp16_azimuth_to_rad(uint16_t azimuth)
{
	return deg_to_rad((azimuth % (360*100)) / 100.0f);
}

void vlp16_parse_data_block(struct vlp16_packet *packet, int block_index)
{
	struct vlp16_data_block *block = &packet->blocks[block_index];

	/* Sanity check */
	if(block->flag != DATA_BLOCK_FLAG) {
		hexdump_uint16("flag: ", block->flag);
		hexdump_uint16("DATA_BLOCK_FLAG: ", DATA_BLOCK_FLAG);
		bail("vlp16_parse_data_block: flag != DATA_BLOCK_FLAG\n");
	}

	float azimuth = vlp16_azimuth_to_rad(block->azimuth);

	/* Sanity check */
	if(azimuth < 0 || azimuth > 2*M_PI) {
		statistics.skipped_azimuth ++;
		return;
	}

	/* Parsed XYZ coordinates will be stored here. */
	double x, y, z;

	for(int i=0; i < CHANNELS_COUNT; i++) {
		struct vlp16_channel_data *channel = &block->channels[i];

		/* Distance is reported to the nearest 2.0mm, meaning that the unsigned
		 * integer value given by the two distance bytes needs to be multiplied
		 * by 2.0mm to calculate the absolute distance to the object. */
		double distance = channel->distance / (double) (DISTANCE_UNIT_FRACTION) * 2.0;

		/* Distances < 1 meter is supposed to be discarded according to
		 * documentation. */
		if(arguments.distance_min != -1) {
			double distance_min = arguments.distance_min / (double) DISTANCE_UNIT_FRACTION;
			if(distance < distance_min) {
				statistics.skipped_r ++;
				continue;
			}
		}

		if(arguments.distance_max != -1) {
			double distance_max = arguments.distance_max / (double) DISTANCE_UNIT_FRACTION;
			if(distance > distance_max) {
				statistics.skipped_r ++;
				continue;
			}
		}

		/* Interpolate azimuth for channels [16..32] */
		if(i == 16) {
			azimuth = vlp16_interpolate_azimuth(packet, block_index);
		}

		/* Transform spherical coordinates to XYZ. */
		int laser_id = i % 16;
		vlp16_transform_coords(laser_id, azimuth, distance, &x, &y, &z);

		/* Move origo. */
		x += arguments.origo_x / (double) DISTANCE_UNIT_FRACTION;
		y += arguments.origo_y / (double) DISTANCE_UNIT_FRACTION;
		z += arguments.origo_z / (double) DISTANCE_UNIT_FRACTION;

		/* Print line */
		switch(arguments.output_format) {
			case OUTPUT_FORMAT_XYZ:
				printf("%f\t%f\t%f\n", x, y, z);
				break;
			case OUTPUT_FORMAT_DEBUG:
				printf("%f\t%f\t%f\t%hhu\t%f\t%d\n", x, y, z, channel->reflectivity, distance, block->azimuth);
				break;
			case OUTPUT_FORMAT_XYZ_BIN:
				/* No support for endian awareness with doubles. May not work on other system! */
				fwrite(&x, sizeof(double), 1, stdout);
				fwrite(&y, sizeof(double), 1, stdout);
				fwrite(&z, sizeof(double), 1, stdout);
				break;
		}
		statistics.points_count ++;
	}
}

char* vlp16_factory_return_mode_string(int return_mode)
{
	switch(return_mode) {
		case FACTORY_RETURN_MODE_STRONGEST:
			return "Strongest Return";
		case FACTORY_RETURN_MODE_LAST:
			return "Last Return";
		case FACTORY_RETURN_MODE_DUAL:
			return "Dual Return";
		default:
			return "Unknown";
	}
}

char* vlp16_factory_source_string(int source)
{
	switch(source) {
		case FACTORY_SOURCE_HDL32E:
			return "HDL-32E";
		case FACTORY_SOURCE_VLP16:
			return "VLP-16";
		default:
			return "Unknown";
	}
}

struct vlp16_packet* vlp16_ntohpacket(const char *buf, const size_t buf_len)
{
	/* Sanity check. */
	if(sizeof(struct vlp16_packet) != buf_len) {
		bail("sizeof(struct vlp16_packet) (%lu) != buf_len (%lu)\n", sizeof(struct vlp16_packet), buf_len);
	}

	struct vlp16_packet *packet = (struct vlp16_packet *) buf;

	for(int i=0; i < DATA_BLOCKS_COUNT; i++) {
		struct vlp16_data_block *block = &packet->blocks[i];

		block->flag = ntohs(block->flag);
		/* NOTE(TS): The documentation states that the azimuth bytes should be
		 * reversed, but in practice this seems not to be true. */
#ifdef AZIMUTH_NET_BYTE_ORDER
		block->azimuth = ntohs(block->azimuth);
#else
		block->azimuth = block->azimuth;
#endif

		// Channel data
		for(int j=0; j < CHANNELS_COUNT; j++) {
			struct vlp16_channel_data *channel = &block->channels[i];
			channel->distance = ntohs(channel->distance);
		}
	}

	packet->timestamp = ntohl(packet->timestamp);

	return packet;
}

void vlp16_parse_packet(const char *buf, const size_t buf_len)
{
	struct vlp16_packet *packet = vlp16_ntohpacket(buf, buf_len);

	for(int i=0; i < DATA_BLOCKS_COUNT; i++) {
		/* In dual return mode, every first block is Last returned and every
		 * other block is {Strongest or 2nd Strongest} return. */
		/* FIXME: for now, just skip every other block. */
		if(packet->factory_return_mode == FACTORY_RETURN_MODE_DUAL) {
			if(i % 2 == 1) {
				continue;
			}
		}
		vlp16_parse_data_block(packet, i);
	}

#ifdef DEBUG_FACTORY
	fprintf(stderr, "factory_return_mode=0x%0x (%s)\n",
			packet->factory_return_mode,
			vlp16_factory_return_mode_string(packet->factory_return_mode));
	fprintf(stderr, "factory_source=0x%0x (%s)\n",
			packet->factory_source,
			vlp16_factory_source_string(packet->factory_source));
#endif
}

void print_statistics()
{
	fprintf(stderr, "\n");
	fprintf(stderr, "Statistics:\n");
	fprintf(stderr, "	Origo: (%d, %d, %d)\n", arguments.origo_x, arguments.origo_y, arguments.origo_z);
	fprintf(stderr, "	Packets received: %d\n", statistics.packets_count);
	fprintf(stderr, "	Valid points: %d\n", statistics.points_count);
	fprintf(stderr, "	Skipped invalid distance: %d\n", statistics.skipped_r);
	fprintf(stderr, "	Skipped invalid azimuth: %d\n", statistics.skipped_azimuth);
	exit(0);
}

int main(int argc, char **argv)
{
	int sockfd;
	socklen_t clientlen;
	struct sockaddr_in serveraddr;
	struct sockaddr_in clientaddr;
	struct hostent *hostp;
	char buf[PACKET_SIZE];
	char *hostaddrp;
	int optval;
	int read;

	/* parse argument options */
	parse_opts(argc, argv);

	signal(SIGINT, print_statistics);

	/* create the server socket */
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if(sockfd < 0) {
		bail("ERROR opening socket\n");
	}

	/* reuse address for fast restart of server */
	optval = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &optval , sizeof(int));

	/* listen for broadcast packets */
	int broadcast = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

	/* server sockaddr_in */
	bzero((char *) &serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serveraddr.sin_port = htons((unsigned short) arguments.port);

	/* bind */
	if(bind(sockfd, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0) {
		bail("ERROR: bind()\n");
	}

	fprintf(stderr, "listening on %d...\n", arguments.port);

	/* main loop: wait for a datagram and parse a packet from it */
	clientlen = sizeof(clientaddr);
	while(1) {
		fprintf(stderr, "recvfrom...\n");

		/* recvfrom: receive a UDP datagram from a client */
		bzero(buf, PACKET_SIZE);
		read = recvfrom(sockfd, buf, PACKET_SIZE, 0, (struct sockaddr *) &clientaddr, &clientlen);
		if(read < 0) {
			bail("ERROR: recvfrom()\n");
		}

		/* Sanity check: packet size is always the same. */
		if(read != PACKET_SIZE) {
			fprintf(stderr, "DEBUG: read (%d) != PACKET_SIZE (%d)\n", read, PACKET_SIZE);
			continue;
		}

		statistics.packets_count ++;
		vlp16_parse_packet(buf, read);

		if(arguments.packets_max != -1 && statistics.packets_count >= arguments.packets_max) {
			fprintf(stderr, "Stopping after %d packets\n", arguments.packets_max);
			break;
		}
	}

	print_statistics();
}
