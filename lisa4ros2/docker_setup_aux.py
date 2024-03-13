lines = ""
with open("src/main/resources/templates/index.html", "r") as f:
    lines = "".join(f.readlines())
    lines = lines.replace("<div class=\"ml-4\"><h4><a href=\"/\">Home</a></h4></div>", "")
with open("src/main/resources/templates/index.html", "w") as f:
    f.write(lines)