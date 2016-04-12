#include <time.h>

#include "ORBVocabulary.h"
using namespace std;

bool load_as_text(ORB_SLAM2::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  bool res = voc->loadFromTextFile(infile);
  printf("Loading fom text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  return res;
}

void load_as_xml(ORB_SLAM2::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->load(infile);
  printf("Loading fom xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void load_as_binary(ORB_SLAM2::ORBVocabulary* voc, const std::string infile) {
  clock_t tStart = clock();
  voc->loadFromBinaryFile(infile);
  printf("Loading fom binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_xml(ORB_SLAM2::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->save(outfile);
  printf("Saving as xml: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_text(ORB_SLAM2::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToTextFile(outfile);
  printf("Saving as text: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

void save_as_binary(ORB_SLAM2::ORBVocabulary* voc, const std::string outfile) {
  clock_t tStart = clock();
  voc->saveToBinaryFile(outfile);
  printf("Saving as binary: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

bool compare(ORB_SLAM2::ORBVocabulary* voc1, ORB_SLAM2::ORBVocabulary* voc2) {
  cout << "compare " << endl;//voc1->m_nodes.size() << endl;


  return true;
}


int main(int argc, char **argv) {
  cout << "BoW load/save benchmark" << endl;
  ORB_SLAM2::ORBVocabulary* voc = new ORB_SLAM2::ORBVocabulary();

  load_as_text(voc, "Vocabulary/ORBvoc.txt");
  
  //save_as_text(voc, "/tmp/foo.txt");
  //save_as_xml(voc, "/tmp/foo.xml");
  save_as_binary(voc, "/tmp/foo.bin");
  save_as_binary(voc, "Vocabulary/ORBvoc.bin");

  //ORB_SLAM2::ORBVocabulary* voc2 = new ORB_SLAM2::ORBVocabulary();
  //load_as_xml(voc2, "/tmp/foo.xml");
  //compare(voc, voc2);

  ORB_SLAM2::ORBVocabulary* voc3 = new ORB_SLAM2::ORBVocabulary();
  load_as_binary(voc3, "/tmp/foo.bin");
  
  save_as_text(voc3, "/tmp/foo2.txt");

  return 0;
}
