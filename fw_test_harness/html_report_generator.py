#!/usr/bin/env python
import argparse
from yattag import Doc
import matplotlib.pyplot as plt, mpld3
import collections


class HtmlReportGenerator:

    """Generates html reports"""

    def __init__(self, args):
        """Constructor"""
        self.args = args
        self.report = "" # the html string
        self.variables = {} # dictionary cotnaining all the result variables
        self.title = ""
        self.plots = collections.OrderedDict() # dictionary of matplotlib plots
        self.plots_mpld3 = collections.OrderedDict() # dictionary of mpld3 plots


    def generate(self):
        """generates the report given the elements added until now"""
        doc, tag, text = Doc().tagtext()

        self.convert_plots()

        with tag('html'):
            with tag('body'):
                with tag('h1', id = 'report_title'):
                    text(self.title)
                with tag('ul', id = 'values'):
                    for k,v in self.variables.items():
                        with tag('li', id = 'values'):
                            text(k + ": " + str(v))
                for k,v in self.plots_mpld3.items():
                    with tag('h2'):
                        text(k)
                    doc.asis(v)
        self.report = doc.getvalue()
        return self.report

    def save(self):
        """saves the report"""
        f = open(self.args["filename_out"], 'w')
        f.write(self.report)
        f.close()

    def convert_plots(self):
        """Convert from matplotlib to mpld3"""
        for k,v in self.plots.items():
            self.plots_mpld3[k] = mpld3.fig_to_html(v)

    def create_add_plot(self, time, plot_values, title):
        """
        Helps generating a matplotlib plot and adds the result to the plots list

        :param time list of time values
        :param plot_values dict of list of values
        :param title a string
        """
        fig = plt.figure(len(self.plots))
        for v in plot_values:
            plt.plot(time, v[1])
        plt.xlabel("time [s]")
        plt.legend([ v[0] for v in plot_values])
        self.plots[title] = fig

    def test(self):
        """Test"""

        # doc, tag, text = Doc().tagtext()

        # with tag('html'):
            # with tag('body'):
                # with tag('p', id = 'main'):
                    # text('some text')
                # with tag('a', href='/my-url'):
                    # text('some link')
        # print(doc.getvalue())
        self.variables = {
            "a": 0.3,
            "b": 0.2}
        self.title = "Test Report"
        f = plt.figure(1)
        plt.plot([1,2,3,4])
        self.plots["Test plot"] = f
        self.generate()
        self.save()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='generates html reports from your results')
    # parser.add_argument('-a', dest='analysis', nargs=1, metavar='analysistype')
    # parser.add_argument('-i', dest='filename', default='./data/datastorage.h5')
    # parser.add_argument('-l', dest='listanalysis', action='store_true')
    parser.add_argument('-o', dest='filename_out', default='report.html')
    args = parser.parse_args()

    rg = HtmlReportGenerator(args)
    rg.test()
