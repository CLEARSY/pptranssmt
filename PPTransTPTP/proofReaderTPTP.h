#ifndef PROOFREADERTPTP_H
#define PROOFREADERTPTP_H

#include<map>
#include<set>
#include<vector>

using namespace std;

/// @brief  ProofReaderTPTP class
/// Object of this class reads a proof in TPTP format and stores it in a data structure
class ProofReaderTPTP {
    public:

        void readProof(const std::string& filename);

    private:
        void readProofLine(const std::string& line){
            if(line.find("cnf") != std::string::npos){
                readCNFLine(line);
            }
            else if(line.find("fof") != std::string::npos){
                readFOFLine(line);
            }
            else if(line.find("tff") != std::string::npos){
                readTFFLine(line);
            }
        }

        void readCNFLine(const std::string& line);
        void readFOFLine(const std::string& line);
        
        void readTFFLine(const std::string& line){

        }

};

#endif // PROOFREADERTPTP_H
