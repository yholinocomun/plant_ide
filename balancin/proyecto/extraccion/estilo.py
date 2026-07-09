"""Estilo UNIFORME para todas las figuras (Python). Mismo look para los 6."""
import matplotlib as mpl
def aplicar():
    mpl.rcParams.update({
      "figure.figsize":(9,7),"figure.dpi":110,"font.size":11,
      "axes.grid":True,"grid.alpha":0.3,"axes.titlesize":12,"axes.titleweight":"bold",
      "lines.linewidth":1.4,"legend.fontsize":9,"savefig.bbox":"tight"})
