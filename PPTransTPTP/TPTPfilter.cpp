#include<iostream>
using std::cin;
using std::cout;

#include<string>
using std::string;
using std::getline;

#include<regex>
using std::regex;
using std::smatch;
using std::regex_search;

int main() 
{
    string line;
    while (getline(cin, line)) {
        // remove everything after the first %
        size_t pos = line.find('%');
        if (pos != string::npos) {
            line.erase(pos);
        }
        // match the line against the regex "file\(\s*'([^']+)'\s*,\s*'([^']+)'\s*\)"
        regex r("file\\(\\s*'([^']+)'\\s*,\\s*'([^']+)'\\s*\\)");
        smatch m;
        if (regex_search(line, m, r)) {
            // if the regex matches, print the first and second capture group
            cout << m[2] << "\n";
        }
    }
}