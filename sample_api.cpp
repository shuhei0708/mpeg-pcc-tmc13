// A functional example of using the G-PCC API with file I/O

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <string>

#include "tmc3/PCCTMC3Encoder.h"
#include "tmc3/PCCTMC3Decoder.h"
#include "tmc3/PCCPointSet.h"
#include "tmc3/ply.h"
#include "tmc3/hls.h"
#include "tmc3/io_tlv.h"

using namespace pcc;

// A callback for the encoder to write the compressed bitstream.
class EncoderCallback : public PCCTMC3Encoder3::Callbacks {
public:
    EncoderCallback(std::ostream& fout) : _fout(fout) {}

    void onOutputBuffer(const PayloadBuffer& buf) override {
        writeTlv(buf, _fout);
    }

    void onPostRecolour(const PCCPointSet3&) override {}

private:
    std::ostream& _fout;
};

// A callback for the decoder to output the reconstructed point cloud.
class DecoderCallback : public PCCTMC3Decoder3::Callbacks {
public:
    void onOutputCloud(const CloudFrame& cloud) override {
        reconstructedCloud = cloud;
    }
    CloudFrame reconstructedCloud;
};


int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input.ply> <output.ply>" << std::endl;
        return 1;
    }

    std::string input_ply_path = argv[1];
    std::string output_ply_path = argv[2];
    std::string compressed_path = "compressed.bin";

    // ========================================================================
    // Encoding
    // ========================================================================
    std::cout << "Encoding " << input_ply_path << " to " << compressed_path << std::endl;

    PCCPointSet3 inputCloud;
    ply::PropertyNameMap propertyNames;
    propertyNames.position = {"x", "y", "z"};
    if (!ply::read(input_ply_path, propertyNames, 1.0, inputCloud)) {
        std::cerr << "Error: couldn't read input file." << std::endl;
        return 1;
    }
    if (inputCloud.getPointCount() == 0) {
        std::cerr << "Error: input file is empty." << std::endl;
        return 1;
    }

    pcc::EncoderParams params;
    params.sps.sps_seq_parameter_set_id = 0;
    params.sps.attributeSets.clear();
    params.gps.gps_geom_parameter_set_id = 0;
    params.gps.gps_seq_parameter_set_id = 0;
    params.autoSeqBbox = true;
    params.trisoupNodeSizesLog2.push_back(0);
    params.gps.qtbt_enabled_flag = true;
    
    // --- Final stable parameters for Octree + RAHT ---
    // Geometry parameters
    params.seqGeomScale = 1.0; 
    params.gps.predgeom_enabled_flag = false;
    params.gps.geom_planar_mode_enabled_flag = false;
    params.gps.geom_angular_mode_enabled_flag = false;
    params.gps.interPredictionEnabledFlag = false;
    params.gps.inferred_direct_coding_mode = 0;
    params.gps.intra_pred_max_node_size_log2 = 6;
    params.gps.neighbour_avail_boundary_log2_minus1 = 7; // 8-1

    // Manually add color attribute encoding settings
    if (inputCloud.hasColors()) {
        AttributeDescription ad;
        ad.attributeLabel = KnownAttributeLabel::kColour;
        ad.attr_num_dimensions_minus1 = 2; // R, G, B
        ad.bitdepth = 8;
        params.sps.attributeSets.push_back(ad);
        params.attributeIdxMap["color"] = 0;

        AttributeParameterSet aps;
        aps.attr_encoding = AttributeEncoding::kRAHTransform;
        aps.init_qp_minus4 = 28; // A safe QP value
        aps.aps_slice_qp_deltas_present_flag = false;
        params.aps.push_back(aps);

        EncoderAttributeParams eap;
        params.attr.push_back(eap);
    }
    
    PCCTMC3Encoder3::deriveParameterSets(&params);


    std::ofstream compressedStream(compressed_path, std::ios::binary);
    if (!compressedStream.is_open()) {
        std::cerr << "Error: couldn't open " << compressed_path << " for writing." << std::endl;
        return 1;
    }

    EncoderCallback encoderCallback(compressedStream);
    PCCTMC3Encoder3 encoder;
    encoder.compress(inputCloud, &params, &encoderCallback);
    compressedStream.close();
    std::cout << "Encoding finished." << std::endl;

    // ========================================================================
    // Decoding
    // ========================================================================
    std::cout << "Decoding " << compressed_path << " to " << output_ply_path << std::endl;
    
    std::ifstream inputStream(compressed_path, std::ios::binary);
    if (!inputStream.is_open()) {
        std::cerr << "Error: couldn't open " << compressed_path << " for reading." << std::endl;
        return 1;
    }

    DecoderParams decoderParams;
    DecoderCallback decoderCallback;
    PCCTMC3Decoder3 decoder(decoderParams);

    // Read the NAL units from the stream and feed to the decoder
    while (inputStream.peek() != EOF) {
        PayloadBuffer buf;
        readTlv(inputStream, &buf);
        if (!inputStream)
            break;
        decoder.decompress(&buf, &decoderCallback);
    }
    
    // Write reconstructed point cloud
    if (decoderCallback.reconstructedCloud.cloud.getPointCount() > 0) {
        ply::write(
            decoderCallback.reconstructedCloud.cloud, propertyNames, 1.0, 
            {0,0,0}, output_ply_path, false);
        std::cout << "Decoding finished. Reconstructed point count: " << decoderCallback.reconstructedCloud.cloud.getPointCount() << std::endl;
    } else {
        std::cerr << "Error: decoding resulted in an empty point cloud." << std::endl;
        return 1;
    }

    return 0;
} 