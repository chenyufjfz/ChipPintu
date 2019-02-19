#include <stdio.h>
#include <string>
using namespace std;
extern int yyparse();
extern int prepare(string filename);

int main(int argc, char *argv[])
{
    prepare("osc.cdl");
    yyparse();
    return 0;
}

