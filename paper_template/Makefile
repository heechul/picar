TARGET = main
SRCS = main.tex

all: 
	pdflatex main
	bibtex main
	pdflatex main


figs/%.pdf: figs/%.eps
	epspdf $< 

clean:
	rm -f *.aux *.bbl *.log *.dvi *.toc .*.swp *~ main.pdf
