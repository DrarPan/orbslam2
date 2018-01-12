#include<DBoW2/FORB.h>
#include<DBoW2/TemplatedVocabulary.h>

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

using namespace std;

string voctxt_file="/home/hitrobot/catkin_ws/src/orbslam2/vocabulary/ORBvoc.txt";
string voctxt_file2="/home/hitrobot/catkin_ws/src/orbslam2/vocabulary/ORBvoc2.txt";
string vocbin_file="/home/hitrobot/catkin_ws/src/orbslam2/vocabulary/ORBvoc.bin";

int main(){
    ORBVocabulary* pVocabulary = new ORBVocabulary();
    ORBVocabulary* pVocabulary2 = new ORBVocabulary();
    bool bVocLoad;
    bVocLoad=pVocabulary->loadFromTextFile(voctxt_file);
    cout<<pVocabulary->m_nodes.size()<<endl;
    cout<<pVocabulary->size()<<endl;
    pVocabulary->saveToBinaryFile(vocbin_file);
    cout<<pVocabulary->m_nodes.size()<<endl;
    cout<<pVocabulary->size()<<endl;
    pVocabulary2->loadFromBinaryFile(vocbin_file);
    cout<<pVocabulary2->m_nodes.size()<<endl;
    cout<<pVocabulary2->size()<<endl;
    pVocabulary2->saveToTextFile(voctxt_file2);
    return -1;
}
