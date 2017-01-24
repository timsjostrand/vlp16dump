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

#define cast_at(type, buf, addr) (*((type *) &buf[addr]))
#define bail(fmt, ...) do { fprintf(stderr, "ERROR: " fmt, ##__VA_ARGS__); exit(1); } while(0)
#define deg_to_rad(deg) (deg * M_PI / 180.0f)
#define rad_to_deg(rad) (rad * 180.0 / M_PI) 

#define FACTORY_RETURN_MODE_STRONGEST	0x37
#define FACTORY_RETURN_MODE_LAST		0x38
#define FACTORY_RETURN_MODE_DUAL		0x39

#define FACTORY_SOURCE_HDL32E			0x21
#define FACTORY_SOURCE_VLP16			0x22

#define DEBUG_DATA_BLOCK
#define DEBUG_FACTORY

#define SKIP_INVALID_DISTANCE
//#define AZIMUTH_NET_BYTE_ORDER

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

#ifdef SKIP_INVALID_DISTANCE
		/* Distances < 1 meter is supposed to be discarded according to
		 * documentation. */
		double distance_min = 1000 / (double) DISTANCE_UNIT_FRACTION;
		if(distance < distance_min) {
			statistics.skipped_r ++;
			continue;
		}
#endif

		/* Interpolate azimuth for channels [16..32] */
		if(i == 16) {
			azimuth = vlp16_interpolate_azimuth(packet, block_index);
		}

		/* Transform spherical coordinates to XYZ. */
		int laser_id = i % 16;
		vlp16_transform_coords(laser_id, azimuth, distance, &x, &y, &z);
		printf("%f\t%f\t%f\t%hhu\t%f\t%d\n", x, y, z, channel->reflectivity, distance, block->azimuth);
		statistics.points_count ++;

		/* TODO: interpolate and update azimuth when i%16==0 */
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
	fprintf(stderr, "	Packets received: %d\n", statistics.packets_count);
	fprintf(stderr, "	Valid points: %d\n", statistics.points_count);
	fprintf(stderr, "	Skipped invalid distance: %d\n", statistics.skipped_r);
	fprintf(stderr, "	Skipped invalid azimuth: %d\n", statistics.skipped_azimuth);
	exit(0);
}

int main(int argc, char **argv)
{
	int sockfd; /* socket */
	int port; /* port to listen on */
	socklen_t clientlen; /* byte size of client's address */
	struct sockaddr_in serveraddr; /* server's addr */
	struct sockaddr_in clientaddr; /* client addr */
	struct hostent *hostp; /* client host info */
	char buf[PACKET_SIZE]; /* message buf */
	char *hostaddrp; /* dotted decimal host addr string */
	int optval; /* flag value for setsockopt */
	int read; /* message byte size */

	signal(SIGINT, print_statistics);

	/* Optionally parse UDP port from arguments. */
	port = DEFAULT_PORT;
	if(argc == 2) {
		port = atoi(argv[1]);
	}

	/*
	 * socket: create the parent socket
	 */
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if(sockfd < 0) {
		bail("ERROR opening socket\n");
	}

	/* setsockopt: Handy debugging trick that lets
	 * us rerun the server immediately after we kill it;
	 * otherwise we have to wait about 20 secs.
	 * Eliminates "ERROR on binding: Address already in use" bail.
	 */
	optval = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &optval , sizeof(int));

	/* listen for broadcast packets */
	int broadcast = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

	/* server sockaddr_in */
	bzero((char *) &serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serveraddr.sin_port = htons((unsigned short)port);

	/* bind */
	if(bind(sockfd, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0) {
		bail("ERROR: bind()\n");
	}

	fprintf(stderr, "listening on %d...\n", port);

	/*
	 * main loop: wait for a datagram, then echo it
	 */
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

		if(statistics.packets_count >= 260) {
			break;
		}

		/* sendto: echo the input back to the client */
#if 0
		read = sendto(sockfd, buf, strlen(buf), 0, (struct sockaddr *) &clientaddr, clientlen);
		if(read < 0) {
			bail("ERROR in sendto\n");
		}
#endif
	}

	print_statistics();
}
