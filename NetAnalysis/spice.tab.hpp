
/* A Bison parser, made by GNU Bison 2.4.1.  */

/* Skeleton interface for Bison's Yacc-like parsers in C
   
      Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
   Free Software Foundation, Inc.
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.
   
   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */


/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     SUBCKT_Action = 258,
     ENDS_Action = 259,
     RLC_Device = 260,
     K_Device = 261,
     IV_Source = 262,
     GE_Source = 263,
     FH_Source = 264,
     X_Device = 265,
     S_Device = 266,
     W_Device = 267,
     MODEL_Action = 268,
     Diode_Device = 269,
     Bipolar_Device = 270,
     JFET_Device = 271,
     MOSFET_Device = 272,
     MESFET_Device = 273,
     End = 274,
     Digits = 275,
     Floats = 276,
     Function = 277,
     TC_Special = 278,
     OFF_Special = 279,
     ON_Special = 280,
     IC_Special = 281,
     TEMP_Special = 282,
     MOS_Special = 283,
     V_Source = 284,
     L_Device = 285,
     Identifier = 286,
     MODEL_Spec = 287,
     Nodes = 288,
     Eol = 289
   };
#endif



#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
{

/* Line 1676 of yacc.c  */
#line 12 "spice.ypp"

	char * str;



/* Line 1676 of yacc.c  */
#line 92 "spice.tab.hpp"
} YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
#endif

extern YYSTYPE yylval;


