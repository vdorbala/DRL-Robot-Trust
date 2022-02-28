import numpy as np
from lxml import etree

def main():
    examples = etree.parse('SingleSentence.xml').getroot()
    # examples = root.find('examples')
    example = examples.findall('example')

    phrases = []

    for phrase in example:
        instruction = phrase.find('instruction')
        phrases.append(str(instruction.text))

    with open('phrases.txt', 'w+') as f:
        for phrase in phrases:
            f.write("%s" % phrase)

if __name__ == '__main__':
    main()