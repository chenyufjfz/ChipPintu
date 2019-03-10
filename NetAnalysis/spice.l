%{
#include <vector>
#include <string>
#include <stdio.h>
#include <string.h>
using namespace std;
#include "spice.tab.hpp"
#include "circuit.h"

#define YY_NEVER_INTERACTIVE 1
extern Circuit * ccc;
extern string yy_filename;
%}

WS       [ \t]
TITLE    [* \t0-9A-Za-z][A-Za-z0-9\- \t;#:=()/\.,*\\]*\r*\n
SPACE    [ \t]
NAME     [A-Z][A-Z0-9]*
IDENT    [A-Za-z_][A-Za-z0-9_\-/]*
IDENT2   [A-Za-z0-9_][A-Za-z0-9_\-/]*
FILE     [/A-Za-z0-9_][/\\:A-Za-z0-9_\.]*
DIGIT    [0-9]
EXPONENT [Ee][+-]?{DIGIT}+
INT      [+-]?{DIGIT}+
FLOAT1   [+-]?{DIGIT}+{EXPONENT}
FLOAT2   [+-]?{DIGIT}*"."{DIGIT}+({EXPONENT})?
FLOAT3   [+-]?{DIGIT}+"."
NUMBER   ({INT}|{FLOAT1}|{FLOAT2}|{FLOAT3}){SFX}?{UNIT}?
SFX      ([tT]|[gG]|[mM]|[kK]|[uU]|[nN]|[pP]|[fF]|[mM][iI][lL]|[mM][eE][gG])
UNIT     ([sS]|[vV]|[aA]|[oO][hH][mM]|[mM][hH][oO]|[fF]|[hH])
NODE     (([A-Za-z0-9_#\+-]|"["|"]")+|[\+]|[-])
EOL      \r*\n
CSFX     [A-Za-z0-9_]*

     /* Source functions */
FUN1     ([dD][eE][cC]|[lL][iI][nN]|[oO][cC][tT]|[dD][cC]|[aA][cC])
FUN2     ([sS][iI][nN]|[pP][uU][lL][sS][eE]|[pP][wW][lL]|[eE][xX][pP])
FUN3     ([dD][iI][sS][tT][oO][fF][12])
FUNCTION ({FUN1}|{FUN2}|{FUN3})
     /* Device Models */
MODEL1   ([rR]|[cC]|[sS][wW]|[cC][sS][wW]|[uU][rR][cC]|[lL][tT][rR][aA])
MODEL2   ([dD]|[nN][pP][nN]|[pP][nN][pP]|[nN][jJ][fF]|[pP][jJ][fF])
MODEL3   ([nN][mM][oO][sS]|[pP][mM][oO][sS]|[nN][mM][fF]|[pP][mM][fF])
MODEL4   ([rR][eE][sS]|[vV][sS][wW][iI][tT][cC][hH])
MODEL    ({MODEL1}|{MODEL2}|{MODEL3}|{MODEL4})
	/* MOS specific device properties */
MOS1     ([lL]|[wW]|[aA][dDsS]|[pP][dDsS]|[nN][rR][dDsS]|[dD][eE][bB][uU][gG])
MOS2     ([rR][gG][eE][oO][mM][oO][dD]|[tT][sSdD]|[mM])
MOS      ({MOS1}|{MOS2})
     /* analog behavioural */
TABLE    [tT][aA][bB][lL][eE]
VALUE    [vV][aA][lL][uU][eE]
FREQ     [fF][rR][eE][qQ]
LAPLACE  [lL][aA][pP][lL][aA][cC][eE]
POLY     [pP][oO][lL][yY]

%x COMMENT STARTUP SUBCKT RLCPROP LREF FUNREF GEVALS FHVALS VSWITCH ISWITCH DEVPROP IGNORE 
%x MODREF1 MODREF2
%option yylineno
%%
<STARTUP,INITIAL>^"*" 	{ /* ignored region begins here */  BEGIN(COMMENT);  }
<STARTUP,INITIAL,DEVPROP,LREF,MODREF1,MODREF2,IGNORE,FUNREF,ISWITCH,VSWITCH,GEVALS,SUBCKT,RLCPROP,FHVALS>[\*\$] {
					    /* ignored inline region begins here (3f5, 2g6 and hspice) */
					    BEGIN(IGNORE);
					  	}
<STARTUP,INITIAL>^"."[sS][uU][bB][cC][kK][tT] {
	    		/* subcircuit definition start */
	    		BEGIN(SUBCKT);
	    		yylval.str = NULL;
	    		return SUBCKT_Action;
				}
<STARTUP>^"."[eE][nN][dD][sS] { 
				/* subcircuit definition end */
	    		BEGIN(SUBCKT);
	    		yylval.str = NULL;
	    		return ENDS_Action;
				}
<STARTUP,INITIAL>^[rRlLcC]{CSFX} { 
				/* R, L and C instances */
				BEGIN(RLCPROP);
				yylval.str = strdup(yytext);
				return RLC_Device;
				}
<STARTUP,INITIAL>^[kK]{CSFX} { 
				/* Mutual inductor instances */
    			BEGIN(LREF);
    			yylval.str = strdup(yytext);
    			return K_Device;
				}
<STARTUP,INITIAL>^[iIvV]{CSFX} {
    			/* voltage and current source instances */
			    BEGIN(FUNREF);
			    yylval.str = strdup(yytext);
			    return IV_Source;
				}
<STARTUP,INITIAL>^[gGeE]{CSFX} {
			    /* voltage controlled source instances */
			    BEGIN(GEVALS);
			    yylval.str = strdup(yytext);
			    return GE_Source;
				}
<STARTUP,INITIAL>^[fFhH]{CSFX} {
			    /* current controlled source instances */
			    BEGIN(FHVALS);
			    yylval.str = strdup(yytext);
			    return FH_Source;
				}
<STARTUP,INITIAL>^[xX]{CSFX} { 
				/* subcircuit instances */
			    BEGIN(STARTUP);
			    yylval.str = strdup(yytext);
			    return X_Device;
				}
<STARTUP,INITIAL>^[sS]{CSFX} { 
				/* voltage controlled switch */
			    BEGIN(VSWITCH);
			    yylval.str = strdup(yytext);
			    return S_Device;
				}
<STARTUP,INITIAL>^[wW]{CSFX} { 
				/* current controlled switch */
			    BEGIN(ISWITCH);
			    yylval.str = strdup(yytext);
			    return W_Device;
				}
<STARTUP,INITIAL>^"."[mM][oO][dD][eE][lL] { 
				/* device Model definitions */
			    BEGIN(MODREF1);
			    yylval.str = NULL;
			    return MODEL_Action;
				}			
<STARTUP>^[dD]{CSFX} { 
				/* diode instances */				    
			    BEGIN(DEVPROP);
			    yylval.str = strdup(yytext);
			    return Diode_Device;
				}
<STARTUP>^[qQ]{CSFX} { 
				/* BJT instances */				    
			    BEGIN(DEVPROP);
			    yylval.str = strdup(yytext);
			    return Bipolar_Device;
				}
<STARTUP>^[jJ]{CSFX} { 
				/* JFET instances */				    
			    BEGIN(DEVPROP);
			    yylval.str = strdup(yytext);
			    return JFET_Device;
				}
<STARTUP>^[mM]{CSFX} { 
				/* MOSFET instances */				    
			    BEGIN(DEVPROP);
			    yylval.str = strdup(yytext);
			    return MOSFET_Device;
				}
<STARTUP>^[zZ]{CSFX} { 
				/* MESFET instances */				    
			    BEGIN(DEVPROP);
			    yylval.str = strdup(yytext);
			    return MESFET_Device;
				}

<STARTUP>"."[Ee][Nn][Dd]{SPACE}*{EOL}* { 
				/* identify end of netlist */
				yylval.str = NULL;
				return End;
				}
<STARTUP,DEVPROP,LREF,FUNREF,ISWITCH,VSWITCH,GEVALS,RLCPROP,FHVALS>{DIGIT}+ {
				/* identify node (containing digits) */
				yylval.str = strdup(yytext);
				return Digits;
				}
<STARTUP,DEVPROP,LREF,FUNREF,GEVALS,RLCPROP,FHVALS>{NUMBER} {
			    /* identify float (any kind) */
			    yylval.str = NULL;
			    return Floats;
				}
<GEVALS,FHVALS>{TABLE}|{VALUE}|{FREQ}|{LAPLACE}|{POLY} {
				/* identify analog behaviour */
				BEGIN(IGNORE);
				}

<FUNREF>{FUNCTION} { 
				/* function specials (e.g. in sources) */
				yylval.str = NULL;
    			return Function;
				}									
<RLCPROP>[tT][cC] { 
				/* a TC special */
				yylval.str = NULL;
				return TC_Special;
				}
<DEVPROP,VSWITCH,ISWITCH>[oO][fF][fF] { 
				/* a device special */
				yylval.str = NULL;
    		    return OFF_Special;
				}		
<VSWITCH,ISWITCH>[oO][nN] { 
				/* a device special */
				yylval.str = NULL;
    			return ON_Special;
				}
<DEVPROP>[iI][cC] { 
				/* another device special */
				yylval.str = NULL;
    			return IC_Special;
				}
<DEVPROP>[tT][eE][mM][pP] { 
				/* another device special */
				yylval.str = NULL;
    			return TEMP_Special;
				}
<DEVPROP>{MOS} { 
				/* another device special */
				yylval.str = NULL;
    			return MOS_Special;
				}							
<ISWITCH,FHVALS>[vV]{CSFX} { 
				/* voltage source references */
				yylval.str = NULL;
			    return V_Source;
				}
<LREF>[lL]{CSFX} { 
				/* inductor references */
				yylval.str = strdup(yytext);
				return L_Device;
				}
<SUBCKT>{IDENT2} { 
				/* arbitrary identifier */
			    BEGIN(STARTUP);
			    yylval.str = strdup(yytext);
			    return Identifier;
				}
<MODREF1>{IDENT2} { 
				/* arbitrary identifier */
    			BEGIN(MODREF2);
    			yylval.str = NULL;
    			return Identifier;
				}
<MODREF2>{MODEL} { 
				/* Model specification specials */
    			BEGIN(STARTUP);
    			yylval.str = NULL;
    			return MODEL_Spec;
				}
<STARTUP,DEVPROP,MODREF2,ISWITCH,VSWITCH,RLCPROP>{IDENT} { 
				/* arbitrary identifier */
				yylval.str = strdup(yytext);
				return Identifier;
				}
<STARTUP,DEVPROP,FUNREF,ISWITCH,VSWITCH,GEVALS,RLCPROP,FHVALS>{NODE} {
    			/* identify node */
    			yylval.str = strdup(yytext);
    			return Nodes;
				}						
<STARTUP,INITIAL,DEVPROP,LREF,IGNORE,FUNREF,ISWITCH,VSWITCH,GEVALS,SUBCKT,RLCPROP,FHVALS>{EOL}  { 
				/* skipping ends here */ 
				BEGIN(STARTUP);  
				yylval.str = NULL;
				return Eol;
				}
<COMMENT>{EOL} { BEGIN(STARTUP);
				/* skipping ends here */ 
				}				
<COMMENT>.      { /* skip any character in here */ }
<IGNORE>.  		{ /* skip any character in here */ }
#.*$ 			{ /* any thing after '#' is a comment */ }
<*>{SPACE}|{EOL}+\+|[()=,] {
				/* skip spaces, '=', '(', ')', ',' and the leading '+' */
				}
<*>[/]			{/* skip '/' */}				
<STARTUP,INITIAL>^"."{CSFX} { 
				/* skipping unknow command */ 
				BEGIN(IGNORE); 
				}
%%

int prepare(string filename, Circuit * c) {
    if ( (yyin = fopen(filename.c_str(),"r")) == 0 ) {
        perror(filename.c_str());
        exit(1);
    }
	yy_filename = filename;
	ccc = c;
    return 0;
}

int yywrap() {
	return 1;
}