/*
 * This code sample muxes an artificially generated video stream with a
 * data stream for camera motion metadata (CAMM) into a single mp4 file.
 * The sample can be built with the "make" command.
 * After building the sample, it can be run as follows:
 *
 *    > ./camm_muxing output_file.mp4
 *
 * Note that this must be run in the sample directory as the "sample.jpg" file.
 *
 * The video stream is a single JPEG file for the entire video duration. It
 * is for illustration purposes only. The calling code is expected to
 * already have the encoded bytes of video frames to mux into the mp4 file.
 */

#include <endian.h>
#include <math.h>
#include <mp4v2/mp4v2.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#define FRAME_WIDTH 2880
#define FRAME_HEIGHT 1800
#define VIDEO_TIME_SECONDS 10
#define VIDEO_TIME_SCALE 90000
#define NUM_CAMM_SAMPLES 200
#define BYTE_BUFFER_SIZE 1000

namespace {
// Number of bytes for the packet type in a CAMM data packet.
const int kCammPacketBytes = 4;
// These sizes are fixed when building with IEEE floating point format.
const int kMetadataTypeSizes[] = {
    // 3 float angle_axis values.
    3 * sizeof(float),
    // 2 int64 values for pixel_exposure_time and rolling_shutter_skew_time.
    2 * sizeof(uint64_t),
    // 3 float gyro values.
    3 * sizeof(float),
    // 3 float acceleration values.
    3 * sizeof(float),
    // 3 float position values.
    3 * sizeof(float),
    // 3 floats for latitude, longitude, and altitude.
    3 * sizeof(float),
    // 3 doubles for time_gps_epoch, latitude, and longitude
    // 1 int32 for gps_fix_type
    // 7 floats for altitude, horizontal_accuracy, vertical_accuracy,
    // vertical_east, vertical_north, vertical_up, and speed_accuracy.
    3 * sizeof(double) + sizeof(uint32_t) + 7 * sizeof(float),
    // 3 floats for magnetic field.
    3 * sizeof(float)};

static uint64_t double_to_bytes(double d) {
  uint64_t result;
  memcpy(&result, &d, 8);
  return htole64(result);
}

static uint32_t float_to_bytes(float f) {
  uint32_t result;
  memcpy(&result, &f, 4);
  return htole32(result);
}

// Writes sample camm data in a byte stream format for a single packet.
// Returns the number of bytes written.
// In real code, you are not required to write all packet types like in this
// example. The pts of each packet also does not need to be equally spaced.
static int WriteSampleCammBytes(void *bytes, int packet_number) {
  uint32_t angle_x, angle_y, angle_z, gyro_x, gyro_y, gyro_z, acceleration_x,
      acceleration_y, acceleration_z, position_x, position_y, position_z,
      latitude32, longitude32, gps_fix_type, horizontal_accuracy_meters,
      vertical_accuracy_meters, vertical_east_velocity_mps,
      vertical_north_velocity_mps, speed_accuracy_mps, altitude,
      magnetic_field_x, magnetic_field_y, magnetic_field_z,
      pixel_exposure_nanoseconds, rolling_shutter_skew_time,
      vertical_up_velocity_mps;
  uint64_t latitude64, longitude64, time_gps_epoch;
  memset(bytes, 0, BYTE_BUFFER_SIZE);
  uint16_t *camm_data = reinterpret_cast<uint16_t *>(bytes);
  uint16_t packet_type = packet_number % 8;
  // The packet type comes after the first 2 reserved bytes.
  camm_data[1] = htole16(packet_type);
  camm_data += 2;
  switch (packet_type) {
    case 0:
      angle_x = float_to_bytes(M_PI / 2);
      angle_y = float_to_bytes(-M_PI / 2);
      angle_z =
          float_to_bytes(fmod(packet_number * M_PI / 20, 2 * M_PI) - M_PI);
      memcpy(camm_data, &angle_x, 4);
      memcpy(camm_data + 2, &angle_y, 4);
      memcpy(camm_data + 4, &angle_z, 4);
      break;
    case 1:
      pixel_exposure_nanoseconds = double_to_bytes(500);
      rolling_shutter_skew_time = double_to_bytes(300);
      memcpy(camm_data, &pixel_exposure_nanoseconds, 8);
      memcpy(camm_data + 4, &rolling_shutter_skew_time, 8);
      break;
    case 2:
      gyro_x = float_to_bytes(M_PI / 20);
      gyro_y = float_to_bytes(2 * M_PI / 20);
      gyro_z = float_to_bytes(3 * M_PI / 20);
      memcpy(camm_data, &gyro_x, 4);
      memcpy(camm_data + 2, &gyro_y, 4);
      memcpy(camm_data + 4, &gyro_z, 4);
      break;
    case 3:
      acceleration_x = float_to_bytes(0.1);
      acceleration_y = float_to_bytes(0.2);
      acceleration_z = float_to_bytes(0.3);
      memcpy(camm_data, &acceleration_x, 4);
      memcpy(camm_data + 2, &acceleration_y, 4);
      memcpy(camm_data + 4, &acceleration_z, 4);
      break;
    case 4:
      position_x = 0;
      position_y = 0;
      position_z = 0;
      memcpy(camm_data, &position_x, 4);
      memcpy(camm_data + 2, &position_y, 4);
      memcpy(camm_data + 4, &position_z, 4);
      break;
    case 5:
      latitude32 = float_to_bytes(37.454356 + .001 * packet_number);
      longitude32 = float_to_bytes(-122.167477 + 0.001 * packet_number);
      altitude = 0;
      memcpy(camm_data, &latitude32, 4);
      memcpy(camm_data + 2, &longitude32, 4);
      memcpy(camm_data + 4, &altitude, 4);
      break;
    case 6:
      time_gps_epoch = double_to_bytes(1500507374.825 + packet_number);
      memcpy(camm_data, &time_gps_epoch, 8);
      camm_data += 4;
      gps_fix_type = 0;
      memcpy(camm_data, &gps_fix_type, 4);
      camm_data += 2;
      latitude64 = double_to_bytes(37.454356 + .001 * packet_number);
      memcpy(camm_data, &latitude64, 8);
      camm_data += 4;
      longitude64 = double_to_bytes(-122.167477 + .001 * packet_number);
      memcpy(camm_data, &longitude64, 8);
      camm_data += 4;
      altitude = 0;
      memcpy(camm_data, &altitude, 4);
      camm_data += 2;
      horizontal_accuracy_meters = float_to_bytes(7.5);
      memcpy(camm_data, &horizontal_accuracy_meters, 4);
      camm_data += 2;
      vertical_accuracy_meters = float_to_bytes(10.5);
      memcpy(camm_data, &vertical_accuracy_meters, 4);
      camm_data += 2;
      vertical_east_velocity_mps = float_to_bytes(1.1);
      memcpy(camm_data, &vertical_east_velocity_mps, 4);
      camm_data += 2;
      vertical_north_velocity_mps = float_to_bytes(1.1);
      memcpy(camm_data, &vertical_north_velocity_mps, 4);
      camm_data += 2;
      vertical_up_velocity_mps = 0;
      memcpy(camm_data, &vertical_up_velocity_mps, 4);
      camm_data += 2;
      speed_accuracy_mps = float_to_bytes(2.5);
      memcpy(camm_data, &speed_accuracy_mps, 4);
      break;
    case 7:
      magnetic_field_x = float_to_bytes(0.01);
      magnetic_field_y = float_to_bytes(0.01);
      magnetic_field_z = float_to_bytes(0.01);
      memcpy(camm_data, &magnetic_field_x, 4);
      memcpy(camm_data + 2, &magnetic_field_y, 4);
      memcpy(camm_data + 4, &magnetic_field_z, 4);
      break;
    default:
      break;
  }
  return kCammPacketBytes + kMetadataTypeSizes[packet_type];
}

static void WriteCammPacket(MP4FileHandle handle, int camm_track_id, int pts,
                            int packet_num, char *buffer) {
  int packet_size = WriteSampleCammBytes(buffer, packet_num);
  if (!MP4WriteSample(handle, camm_track_id,
                      reinterpret_cast<uint8_t *>(buffer), packet_size,
                      /* duration */ 0,
                      /* rendering_offset */ pts,
                      /* is_sync_sample */ true)) {
    printf("Failed to write camm track sample\n");
    exit(1);
  }
}

static void WriteVideoFrame(MP4FileHandle handle, int video_track_id,
                            const std::vector<char> &jpg_bytes, int pts,
                            int duration) {
  // This code sample doesn't illustrate how to encode video. An encoder
  // for h264 or another format should be used to do that, and then call
  // MP4WriteSample with the bytes returned from the encoder.
  if (!MP4WriteSample(handle, video_track_id,
                      reinterpret_cast<const uint8_t *>(jpg_bytes.data()),
                      /* num_bytes */ jpg_bytes.size(),
                      /* duration */ VIDEO_TIME_SCALE,
                      /* rendering_offset */ pts,
                      /* is_sync_sample */ true)) {
    printf("Failed to write mp4 sample to video track\n");
    exit(1);
  }
}

static void WriteStreams(MP4FileHandle handle, int video_track_id,
                         const std::vector<char> &jpg_bytes,
                         int camm_track_id) {
  char camm_buffer[BYTE_BUFFER_SIZE];
  int video_pts = 0;
  int camm_pts = 0;
  int camm_packet_num = 0;
  int max_pts = VIDEO_TIME_SECONDS * VIDEO_TIME_SCALE;
  while (video_pts < max_pts && camm_pts < max_pts) {
    if (video_pts <= camm_pts) {
      WriteVideoFrame(handle, video_track_id, jpg_bytes, video_pts,
                      VIDEO_TIME_SCALE);
      video_pts += VIDEO_TIME_SCALE;
    } else {
      WriteCammPacket(handle, camm_track_id, camm_pts, camm_packet_num,
                      camm_buffer);
      camm_pts += max_pts / NUM_CAMM_SAMPLES;
      camm_packet_num++;
    }
  }
}

static int AddVideoTrack(MP4FileHandle handle) {
  printf("Adding video track\n");
  int video_track_id = MP4AddVideoTrack(
      handle, VIDEO_TIME_SCALE, VIDEO_TIME_SCALE * VIDEO_TIME_SECONDS,
      FRAME_WIDTH, FRAME_HEIGHT, MP4_JPEG_VIDEO_TYPE);
  if (video_track_id == MP4_INVALID_TRACK_ID) {
    printf("Can't add video track\n");
    exit(1);
  }
  return video_track_id;
}

static int AddCammTrack(MP4FileHandle handle) {
  printf("Adding camm track\n");
  int data_track_id = MP4AddTrack(handle, "camm", VIDEO_TIME_SCALE);
  if (data_track_id == MP4_INVALID_TRACK_ID) {
    printf("Can't add camm data track\n");
    exit(1);
  }
  if (!MP4SetTrackName(handle, data_track_id, "camm")) {
    printf("Couldn't set the track name of the CAMM track.\n");
    exit(1);
  }
  return data_track_id;
}

// Reads the contents of the sample jpeg file into memory.
static std::vector<char> ReadSampleFileBytes() {
  struct stat buffer;
  const char *filename = "sample.jpg";
  if (stat(filename, &buffer) != 0) {
    printf(
        "Sample file 'sample.jpg' doesn't exist\n"
        "Make sure it is in the directory and then run the program\n"
        "again.\n");
    exit(1);
  }
  std::ifstream ifs(filename, std::ios::binary | std::ios::ate);
  std::ifstream::pos_type pos = ifs.tellg();
  std::vector<char> result(pos);
  ifs.seekg(0, std::ios::beg);
  ifs.read(&result[0], pos);
  return result;
}

static void VerifyProgramArgs(int argc, char **argv) {
  if (argc != 2) {
    printf(
        "Usage: %s file.mp4\n"
        "file.mp4 is the output location of the generated video\n",
        argv[0]);
    exit(1);
  }
}
}  // namespace

int main(int argc, char **argv) {
#ifndef __STDC_IEC_559__
  fprintf(stderr,
          "Please ensure your compiler uses IEEE 754 \n"
          "floating point representation. If so, you may safely comment out \n"
          "this guard.\n");
  exit(1);
#endif

  VerifyProgramArgs(argc, argv);
  std::vector<char> jpg_bytes = ReadSampleFileBytes();
  MP4FileHandle handle = MP4Create(argv[1], /* flags */ 0);
  int video_track_id = AddVideoTrack(handle);
  int camm_track_id = AddCammTrack(handle);

  WriteStreams(handle, video_track_id, jpg_bytes, camm_track_id);

  printf("Dumping MP4 file information:\n");
  MP4Dump(handle);
  MP4Close(handle);
  printf("Done!\n");
  return 0;
}
