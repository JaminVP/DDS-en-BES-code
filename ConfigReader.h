/**
* @file ConfigReader.h
* @ingroup DimensionalDataSource
* @author Jamin Van Parys (jaminvp@gmail.com)
* @date 27/06/2015
* @brief Utility Class which gives access to certain configuration constants
* @version 1.0
*/

#ifndef DimensionalDataSource_ConfigReader_h
#define DimensionalDataSource_ConfigReader_h
#include <iostream>
#include <map>
#include <string>
#include <fstream>
h,hlf,h<teml
using namespace std;

typedef map<const string,string> CfgMap;

/**

* @brief Simple Class that allows parsing in configuration key-value pairs from a file into a string/string map
*/
class ConfigReader{
	public:

	string& operator[] (const string& s);

	ConfigReader(const string configFilePath="config.txt"){
		string s;
		ifstream read;
		read.open(configFilePath);
		if (!read.is_open()) {
			cerr << "Can't open " << configFilePath << "!!\n";
		}
		while(getline(read,s)){
			cfg[s.substr(0,s.find('='))]=s.substr(s.find('=')+1);
		}
		read.close();

	}

	void showSettings(){
		cout << "All settings:\n";
		for(CfgMap::iterator i=cfg.begin();i!=cfg.end();i++) {
			cout << "Setting \"" << i->first << "\" has value \"" << i->second << '\"' << endl ;
		}
		cout<<endl;
	}
private:
	CfgMap cfg;


};

string& ConfigReader::operator[] (const string& s)
{
	return cfg[s];
}

#endif
