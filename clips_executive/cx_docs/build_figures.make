TEXDIR := figure_sources
FIGDIR := figures

LATEXMK := latexmk
PDF2SVG := pdf2svg

TEXFILES := \
	gym_comm_action_masking.tex \
	gym_comm_end_training.tex \
	gym_comm_exec.tex \
	gym_comm_legend.tex \
	gym_comm_obs_action_space.tex \
	gym_comm_reset.tex \
	gym_comm_get_status.tex \
	gym_comm_step.tex

PDFS := $(TEXFILES:%.tex=$(TEXDIR)/%.pdf)
SVGS := $(TEXFILES:%.tex=$(FIGDIR)/%.svg)

.PHONY: all clean

all: $(SVGS)

# Compile each TikZ file to PDF
$(TEXDIR)/%.pdf: $(TEXDIR)/%.tex
	cd $(TEXDIR) && $(LATEXMK) -pdf -interaction=nonstopmode $*.tex
	cd $(TEXDIR) && $(LATEXMK) -c $*.tex

# Convert PDF → SVG (stable)
$(FIGDIR)/%.svg: $(TEXDIR)/%.pdf
	mkdir -p $(FIGDIR)
	$(PDF2SVG) $< $@

clean:
	# Remove only generated PDFs and SVGs
	rm -f $(PDFS) $(SVGS)
	# Clean aux files produced by latexmk
	cd $(TEXDIR) && $(LATEXMK) -c
